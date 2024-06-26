# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: Action.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='Action.proto',
  package='gymfc.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x0c\x41\x63tion.proto\x12\ngymfc.msgs\"~\n\x06\x41\x63tion\x12\x11\n\x05motor\x18\x01 \x03(\x02\x42\x02\x10\x01\x12<\n\rworld_control\x18\x02 \x01(\x0e\x32\x1f.gymfc.msgs.Action.WorldControl:\x04STEP\"#\n\x0cWorldControl\x12\x08\n\x04STEP\x10\x00\x12\t\n\x05RESET\x10\x01')
)



_ACTION_WORLDCONTROL = _descriptor.EnumDescriptor(
  name='WorldControl',
  full_name='gymfc.msgs.Action.WorldControl',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STEP', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RESET', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=119,
  serialized_end=154,
)
_sym_db.RegisterEnumDescriptor(_ACTION_WORLDCONTROL)


_ACTION = _descriptor.Descriptor(
  name='Action',
  full_name='gymfc.msgs.Action',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='motor', full_name='gymfc.msgs.Action.motor', index=0,
      number=1, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=_b('\020\001'), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='world_control', full_name='gymfc.msgs.Action.world_control', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _ACTION_WORLDCONTROL,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=28,
  serialized_end=154,
)

_ACTION.fields_by_name['world_control'].enum_type = _ACTION_WORLDCONTROL
_ACTION_WORLDCONTROL.containing_type = _ACTION
DESCRIPTOR.message_types_by_name['Action'] = _ACTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Action = _reflection.GeneratedProtocolMessageType('Action', (_message.Message,), dict(
  DESCRIPTOR = _ACTION,
  __module__ = 'Action_pb2'
  # @@protoc_insertion_point(class_scope:gymfc.msgs.Action)
  ))
_sym_db.RegisterMessage(Action)


_ACTION.fields_by_name['motor']._options = None
# @@protoc_insertion_point(module_scope)
