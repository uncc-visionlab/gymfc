# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: surface.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import friction_pb2 as friction__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='surface.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_pb=_b('\n\rsurface.proto\x12\x0bgazebo.msgs\x1a\x0e\x66riction.proto\"\xc9\x02\n\x07Surface\x12\'\n\x08\x66riction\x18\x01 \x01(\x0b\x32\x15.gazebo.msgs.Friction\x12\x1f\n\x17restitution_coefficient\x18\x02 \x01(\x01\x12\x18\n\x10\x62ounce_threshold\x18\x03 \x01(\x01\x12\x10\n\x08soft_cfm\x18\x04 \x01(\x01\x12\x10\n\x08soft_erp\x18\x05 \x01(\x01\x12\n\n\x02kp\x18\x06 \x01(\x01\x12\n\n\x02kd\x18\x07 \x01(\x01\x12\x0f\n\x07max_vel\x18\x08 \x01(\x01\x12\x11\n\tmin_depth\x18\t \x01(\x01\x12\x1f\n\x17\x63ollide_without_contact\x18\n \x01(\x08\x12\'\n\x1f\x63ollide_without_contact_bitmask\x18\x0b \x01(\r\x12\x17\n\x0f\x63ollide_bitmask\x18\x0c \x01(\r\x12\x17\n\x0f\x65lastic_modulus\x18\r \x01(\x01')
  ,
  dependencies=[friction__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SURFACE = _descriptor.Descriptor(
  name='Surface',
  full_name='gazebo.msgs.Surface',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='friction', full_name='gazebo.msgs.Surface.friction', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='restitution_coefficient', full_name='gazebo.msgs.Surface.restitution_coefficient', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bounce_threshold', full_name='gazebo.msgs.Surface.bounce_threshold', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='soft_cfm', full_name='gazebo.msgs.Surface.soft_cfm', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='soft_erp', full_name='gazebo.msgs.Surface.soft_erp', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kp', full_name='gazebo.msgs.Surface.kp', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kd', full_name='gazebo.msgs.Surface.kd', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_vel', full_name='gazebo.msgs.Surface.max_vel', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_depth', full_name='gazebo.msgs.Surface.min_depth', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='collide_without_contact', full_name='gazebo.msgs.Surface.collide_without_contact', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='collide_without_contact_bitmask', full_name='gazebo.msgs.Surface.collide_without_contact_bitmask', index=10,
      number=11, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='collide_bitmask', full_name='gazebo.msgs.Surface.collide_bitmask', index=11,
      number=12, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='elastic_modulus', full_name='gazebo.msgs.Surface.elastic_modulus', index=12,
      number=13, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=47,
  serialized_end=376,
)

_SURFACE.fields_by_name['friction'].message_type = friction__pb2._FRICTION
DESCRIPTOR.message_types_by_name['Surface'] = _SURFACE

Surface = _reflection.GeneratedProtocolMessageType('Surface', (_message.Message,), dict(
  DESCRIPTOR = _SURFACE,
  __module__ = 'surface_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.Surface)
  ))
_sym_db.RegisterMessage(Surface)


# @@protoc_insertion_point(module_scope)
