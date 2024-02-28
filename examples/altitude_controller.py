#!/usr/bin/env python

import argparse
import os
import sys
import time
import matplotlib.pyplot as plt
import numpy as np

import gym
from gym.envs.registration import register
from gymfc_nf.envs.base import BaseEnv


def state_degrees_error_deltaerror(local):
    error_delta = local.measured_error - local.last_measured_error
    return np.concatenate([local.measured_error, error_delta])


register(
    id="gymfc_quadrotor_env_v1",
    entry_point='altitude_controller:StepEnv',
    kwargs={
        "max_rate": 100,
        "state_fn": state_degrees_error_deltaerror,
        "pulse_width": 2.048,
        "max_sim_time": 4.608
    }
)


class StepEnv(BaseEnv):
    def __init__(self, pulse_width=1, max_rate=100, state_fn=None, max_sim_time=1):
        """Create a reinforcement learning environment that generates step input
        setpoints. Technically this is a multi-axis singlet input, the
        terminology in this package needs to be updated to reflect flight test
        maneuvers.

        This environment was created to teach an agent how to respond to
        worst-case inputs, that is, step inputs in which there is a request for
        immediate change in the target angular velocity.

        Start at zero deg/s to establish an initial condition and teach the
        agent to idle. Sample random input and hold for pulse_width, then
        return to zero deg/s to allow system to settle.

        Args:
            pulse_width: Number of seconds the step is held at the target
                setpoint.
            max_rate: Max angular rate to sample from, or in the case of a
                normal distribution, the mean.
            state_fn: See BaseEnv
            max_sim_time: See BaseEnv
        """

        super().__init__(max_sim_time=max_sim_time, state_fn=state_fn)

        self.pulse_width = pulse_width
        self.max_rate = max_rate

        self.rising = True
        self.outputs = []
        self.angular_rate_sp = np.zeros(3)
        self.next_pulse_time = 0.512

    def update_setpoint(self):
        if self.sim_time > self.next_pulse_time:
            if (self.angular_rate_sp == np.zeros(3)).all():
                self.angular_rate_sp = self.generated_input
                self.next_pulse_time += self.pulse_width
            else:
                self.angular_rate_sp = np.zeros(3)
                self.next_pulse_time += self.pulse_width
            self.rising = False

    def reset(self):
        self.rising = True
        self.outputs = []
        self.angular_rate_sp = np.zeros(3)
        self.next_pulse_time = 0.512
        # Define the singlet input in the beginning so it can be overriden
        # externally if needed for testing.
        self.generated_input = self.sample_target()
        return super().reset()

    def sample_target(self):
        """Sample a random angular velocity setpoint """
        return self.np_random.normal(0, self.max_rate, size=3)


def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


class PID(object):
    """A simple PID controller."""

    def __init__(
            self,
            Kp=1.0,
            Ki=0.0,
            Kd=0.0,
            setpoint=0,
            sample_time=0.01,
            output_limits=(None, None),
            auto_mode=True,
            proportional_on_measurement=False,
            differential_on_measurement=True,
            error_map=None,
            time_fn=None,
            starting_output=0.0,
    ):
        """
        Initialize a new PID controller.

        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param differential_on_measurement: Whether the differential term should be calculated on
            the input directly rather than on the error (which is the traditional way).
        :param error_map: Function to transform the error value in another constrained value.
        :param time_fn: The function to use for getting the current time, or None to use the
            default. This should be a function taking no arguments and returning a number
            representing the current time. The default is to use time.monotonic() if available,
            otherwise time.time().
        :param starting_output: The starting point for the PID's output. If you start controlling
            a system that is already at the setpoint, you can set this to your best guess at what
            output the PID should give when first calling it to avoid the PID outputting zero and
            moving the system away from the setpoint.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.differential_on_measurement = differential_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_error = None
        self._last_input = None

        if time_fn is not None:
            # Use the user supplied time function
            self.time_fn = time_fn
        else:
            import time

            try:
                # Get monotonic time to ensure that time deltas are always positive
                self.time_fn = time.monotonic
            except AttributeError:
                # time.monotonic() not available (using python < 3.3), fallback to time.time()
                self.time_fn = time.time

        self.output_limits = output_limits
        self.reset()

        # Set initial state of the controller
        self._integral = _clamp(starting_output, output_limits)

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.

        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).

        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = self.time_fn()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)
        d_error = error - (self._last_error if (self._last_error is not None) else error)

        # Check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

        if self.differential_on_measurement:
            self._derivative = -self.Kd * d_input / dt
        else:
            self._derivative = self.Kd * d_error / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_error = error
        self._last_time = now

        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r}, '
            'differential_on_measurement={self.differential_on_measurement!r}, '
            'error_map={self.error_map!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.

        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.

        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # Switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if (last_output is not None) else 0
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).

        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = _clamp(self._integral, self.output_limits)

        self._last_time = self.time_fn()
        self._last_output = None
        self._last_input = None


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Evaluate each algorithm")
    parser.add_argument('--model', default="../gazebo/models/iris/model.sdf",
                        help="File path of the NF1 twin/model SDF.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    args = parser.parse_args()

    verbose = True
    render = True
    current_altitude = 487.9

    # Need to set the aircraft model after we create the environment because
    # the model is unique and can't be hardcoded in the gym init.
    # env = gym.make(args.gym_id)
    env = gym.make("gymfc_quadrotor_env_v1")
    env.setup(args.model, args.gymfc_config)
    env.seed(args.seed)
    env.reset()
    if render == True:
        env.render()
    env.verbose = verbose

    pid = PID(Kp=.05, Ki=.0005, Kd=0.00, sample_time=env.stepsize, setpoint=0.7)
    pid.output_limits = (-1000, 1000)
    motor_rpm_sp = 0

    # let the quad settle onto the ground
    while env.sim_time < 3:
        ob, reward, done, _ = env.step_basic([motor_rpm_sp] * 4)

    current_step = 0
    pid.setpoint = current_altitude + 0.7
    # spin up the motors
    while env.sim_time < 60:
        delta_motor_rpm_sp = pid(current_altitude, env.stepsize)
        motor_rpm_sp += delta_motor_rpm_sp
        ob, reward, done, _ = env.step_basic([motor_rpm_sp] * 4)
        motor_rpm_actual = 10 * np.array(ob.esc_motor_angular_velocity[0:4])
        if current_step % 1000 == 0:
            print("step={} height_sp={:.2f} h={:.2f} ".format(current_step, pid.setpoint, current_altitude) +
                  "setpoint_rpm={:.2f} actual_rpm={:.2f} ".format(motor_rpm_sp, motor_rpm_actual[0]) +
                  "delta_rpm_sp={:.2f}".format(delta_motor_rpm_sp))
        # if done:
        #     break
        current_altitude = ob.gt_wgs84_pos[2]
        current_step += 1

    env.close()

    # plt.plot(x, y, label='measured')
    # plt.plot(x, setpoint, label='target')
    # plt.xlabel('time')
    # plt.ylabel('temperature')
    # plt.legend()
    # if os.getenv('NO_DISPLAY'):
    #     # If run in CI the plot is saved to file instead of shown to the user
    #     plt.savefig(f"result-py{'.'.join([str(x) for x in sys.version_info[:2]])}.png")
    # else:
    #     plt.show()
