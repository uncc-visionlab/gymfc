import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv
import numpy as np
import time
import matplotlib.pyplot as plt


def step_input(env, output_file, max_command, max_sim_time, input_type="step"):
    """
        Args:
            env
            output_file
            max_command
    """
    ob = env.reset()
    command = max_command
    data = []

    peak_time = max_sim_time / 2.0
    ramp_steps = peak_time / env.stepsize
    command_step = 1 / ramp_steps
    current_step = 0
    while True:
        if input_type == "step":
            if env.sim_time > peak_time:
                command = np.zeros(env.motor_count, dtype=float)
        else:
            if env.sim_time < peak_time:
                command = max_command * env.sim_time / peak_time
            else:
                command = max_command - max_command * (env.sim_time - peak_time) / (2 * peak_time)
        ob = env.step_sim(command)
        if current_step % 1000 == 0:
            print("step={} command = {} rpm/rotorVelocitySlowdownSim = {}".format(current_step, command,
                                                                                  ob.esc_motor_angular_velocity))
        data.append([env.sim_time, command[0],
                     *ob.esc_motor_angular_velocity,
                     *ob.force, env.motor_count])
        current_step += 1
        if env.sim_time > max_sim_time:
            break

    env.close()
    np.savetxt(output_file, data, header="time,command,velocity,force,torque", delimiter=",")


class Sim(FlightControlEnv):

    def __init__(self, aircraft_config, config_filepath=None, verbose=False):
        super().__init__(aircraft_config, config_filepath=config_filepath, verbose=verbose)


if __name__ == "__main__":

    parser = argparse.ArgumentParser("A dyno emulator for validating motor models in simulation.")
    parser.add_argument('output', help="Output file.")
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('value', nargs='+', type=float, help="Throttle value for the motor.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--delay', default=0, type=float, help="Second delay between steps for debugging purposes.")
    parser.add_argument('--max-sim-time', default=1, type=float, help="Time in seconds the sim should run for.")
    parser.add_argument('--verbose', action="store_true", help="Provide additional logs from Gazebo.")
    parser.add_argument('--render', action="store_true", help="Display the Gazebo client.")
    parser.add_argument('--input-type', default="step", help="step or ramp")

    args = parser.parse_args()

    env = Sim(args.aircraftconfig, config_filepath=args.gymfc_config, verbose=args.verbose)
    if args.render:
        env.render()
    step_input(env, args.output, np.array(args.value), args.max_sim_time, args.input_type)
