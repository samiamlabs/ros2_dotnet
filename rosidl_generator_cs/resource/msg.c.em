// generated from rosidl_generator_cs/resource/msg.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>_s.c files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - message (IdlMessage, with structure containing names, types and members)
@#  - idl_type_to_c, value_to_c, idl_structure_type_to_c_typename - converting names, types and values from idl to c
@#######################################################################

@{
from rosidl_generator_c import idl_type_to_c
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import BasicType
}@

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

@{
msg_typename = idl_structure_type_to_c_typename(message.structure.namespaced_type)
include_path = "/".join(include_parts)
have_not_included_string = True
}@
#include <@(include_path)__struct.h>
#include <@(include_path)__functions.h>
@[for member in message.structure.members]@
@[  if isinstance(member.type, AbstractGenericString) and have_not_included_string]@
@{have_not_included_string = False}@
#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

@[  end if]@
@[end for]@


@[for member in message.structure.members]@
@[  if isinstance(member.type, BasicType) or isinstance(member.type, AbstractGenericString)]@
ROSIDL_GENERATOR_C_EXPORT
@(get_c_type(member.type)) @(msg_typename)_native_read_field_@(member.name)(void *message_handle)
{
  @(msg_typename) *ros_message = (@(msg_typename) *)message_handle;
@[    if  isinstance(member.type, AbstractGenericString)]@
  return ros_message->@(member.name).data;
@[    else]@
  return ros_message->@(member.name);
@[    end if]@
}
@[  end if]@
@[end for]@

@[for member in message.structure.members]@
@[  if isinstance(member.type, BasicType) or isinstance(member.type, AbstractGenericString)]@
ROSIDL_GENERATOR_C_EXPORT
void @(msg_typename)_native_write_field_@(member.name)(void *message_handle, @(get_c_type(member.type)) value)
{
  @(msg_typename) *ros_message = (@(msg_typename) *)message_handle;
@[    if  isinstance(member.type, AbstractGenericString)]@
  rosidl_generator_c__String__assign(
    &ros_message->@(member.name), value);
@[    else]@
  ros_message->@(member.name) = value;
@[    end if]@
}
@[  end if]@
@[end for]@
