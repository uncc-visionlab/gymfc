# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rest_response.proto

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
  name='rest_response.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_pb=_b('\n\x13rest_response.proto\x12\x0bgazebo.msgs\"\x8a\x01\n\x0cRestResponse\x12\n\n\x02id\x18\x01 \x01(\r\x12,\n\x04type\x18\x02 \x02(\x0e\x32\x1e.gazebo.msgs.RestResponse.Type\x12\x0b\n\x03msg\x18\x03 \x01(\t\"3\n\x04Type\x12\x0b\n\x07SUCCESS\x10\x01\x12\x07\n\x03\x45RR\x10\x02\x12\t\n\x05LOGIN\x10\x03\x12\n\n\x06LOGOUT\x10\x04')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_RESTRESPONSE_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='gazebo.msgs.RestResponse.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SUCCESS', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOGIN', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOGOUT', index=3, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=124,
  serialized_end=175,
)
_sym_db.RegisterEnumDescriptor(_RESTRESPONSE_TYPE)


_RESTRESPONSE = _descriptor.Descriptor(
  name='RestResponse',
  full_name='gazebo.msgs.RestResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='gazebo.msgs.RestResponse.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='gazebo.msgs.RestResponse.type', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg', full_name='gazebo.msgs.RestResponse.msg', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _RESTRESPONSE_TYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=37,
  serialized_end=175,
)

_RESTRESPONSE.fields_by_name['type'].enum_type = _RESTRESPONSE_TYPE
_RESTRESPONSE_TYPE.containing_type = _RESTRESPONSE
DESCRIPTOR.message_types_by_name['RestResponse'] = _RESTRESPONSE

RestResponse = _reflection.GeneratedProtocolMessageType('RestResponse', (_message.Message,), dict(
  DESCRIPTOR = _RESTRESPONSE,
  __module__ = 'rest_response_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.RestResponse)
  ))
_sym_db.RegisterMessage(RestResponse)


# @@protoc_insertion_point(module_scope)
