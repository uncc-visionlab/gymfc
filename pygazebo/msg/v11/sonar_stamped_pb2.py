# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sonar_stamped.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import time_pb2 as time__pb2
from . import sonar_pb2 as sonar__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sonar_stamped.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_pb=_b('\n\x13sonar_stamped.proto\x12\x0bgazebo.msgs\x1a\ntime.proto\x1a\x0bsonar.proto\"R\n\x0cSonarStamped\x12\x1f\n\x04time\x18\x01 \x02(\x0b\x32\x11.gazebo.msgs.Time\x12!\n\x05sonar\x18\x02 \x02(\x0b\x32\x12.gazebo.msgs.Sonar')
  ,
  dependencies=[time__pb2.DESCRIPTOR,sonar__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SONARSTAMPED = _descriptor.Descriptor(
  name='SonarStamped',
  full_name='gazebo.msgs.SonarStamped',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time', full_name='gazebo.msgs.SonarStamped.time', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sonar', full_name='gazebo.msgs.SonarStamped.sonar', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=61,
  serialized_end=143,
)

_SONARSTAMPED.fields_by_name['time'].message_type = time__pb2._TIME
_SONARSTAMPED.fields_by_name['sonar'].message_type = sonar__pb2._SONAR
DESCRIPTOR.message_types_by_name['SonarStamped'] = _SONARSTAMPED

SonarStamped = _reflection.GeneratedProtocolMessageType('SonarStamped', (_message.Message,), dict(
  DESCRIPTOR = _SONARSTAMPED,
  __module__ = 'sonar_stamped_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.SonarStamped)
  ))
_sym_db.RegisterMessage(SonarStamped)


# @@protoc_insertion_point(module_scope)
