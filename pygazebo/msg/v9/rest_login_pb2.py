# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rest_login.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='rest_login.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_pb=_b('\n\x10rest_login.proto\x12\x0bgazebo.msgs\"H\n\tRestLogin\x12\n\n\x02id\x18\x01 \x01(\r\x12\x0b\n\x03url\x18\x02 \x02(\t\x12\x10\n\x08username\x18\x03 \x01(\t\x12\x10\n\x08password\x18\x04 \x01(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_RESTLOGIN = _descriptor.Descriptor(
  name='RestLogin',
  full_name='gazebo.msgs.RestLogin',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='gazebo.msgs.RestLogin.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='url', full_name='gazebo.msgs.RestLogin.url', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='username', full_name='gazebo.msgs.RestLogin.username', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='password', full_name='gazebo.msgs.RestLogin.password', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=33,
  serialized_end=105,
)

DESCRIPTOR.message_types_by_name['RestLogin'] = _RESTLOGIN

RestLogin = _reflection.GeneratedProtocolMessageType('RestLogin', (_message.Message,), dict(
  DESCRIPTOR = _RESTLOGIN,
  __module__ = 'rest_login_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.RestLogin)
  ))
_sym_db.RegisterMessage(RestLogin)


# @@protoc_insertion_point(module_scope)
