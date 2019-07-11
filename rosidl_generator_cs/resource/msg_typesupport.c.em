// generated from rosidl_generator_cs/resource/msg_typesupport.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg_pkg>_s.ep.<typesupport_impl>_c.c files
@#
@# Context:
@#  - package_name
@#  - interface_path
@#  - message (IdlMessage structure)
@#######################################################################
@

#include <stdbool.h>
#include <stdint.h>
#include <rosidl_generator_c/message_type_support_struct.h>
#include <rosidl_generator_c/visibility_control.h>

@{
includes = {}
type_name = message.structure.namespaced_type.name
key = '%s/%s/%s' % (package_name, interface_path, type_name)
includes[key + '_support'] = '#include <%s__type_support.h>' % key
includes[key + '_struct'] = '#include <%s__struct.h>' % key
includes[key + '_functions'] = '#include <%s__functions.h>' % key
}@
@[for v in sorted(includes.values())]@
@(v)
@[end for]@

ROSIDL_GENERATOR_C_EXPORT
void * @(package_name)__@(interface_path)__@(type_name)__get_type_support()
{
    return (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(@(package_name), @(interface_path), @(type_name));
}
@[end for]@
