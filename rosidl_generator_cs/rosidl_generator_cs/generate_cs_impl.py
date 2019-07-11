# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import defaultdict
import os
from rosidl_cmake import generate_files
from rosidl_generator_c import idl_type_to_c
from rosidl_generator_c import value_to_c
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import UnboundedSequence
import logging
import sys

def generate_cs(generator_arguments_file, typesupport_impls):
    type_support_impl_by_filename = {
        '_%s_s.ep.{0}.c'.format(impl): impl for impl in typesupport_impls
    }

    mapping = {
        'idl.cs.em': '%s.cs',
        'idl.c.em': '%s_s.c'
    }

    print("Type_support mapping " + str(type_support_impl_by_filename), file=sys.stderr)
    additional_context = {
        'escape_string' : escape_string,
        'get_field_name' : get_field_name,
        'get_dotnet_type' : get_dotnet_type
    }

    generate_files(generator_arguments_file, mapping, additional_context)
    for type_support in type_support_impl_by_filename.keys():
        typemapping = { 'idl_typesupport.c.em': type_support }
        generate_files(generator_arguments_file, typemapping)


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s

def get_builtin_dotnet_type(type_, use_primitives=True):
    if type_ == 'bool':
        return 'bool' if use_primitives else 'System.Boolean'

    if type_ == 'byte':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'char':
        return 'char' if use_primitives else 'System.Char'

    if type_ == 'float32':
        return 'float' if use_primitives else 'System.Single'

    if type_ == 'float64':
        return 'double' if use_primitives else 'System.Double'

    if type_ == 'int8':
        return 'sbyte' if use_primitives else 'System.Sbyte'

    if type_ == 'uint8':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'int16':
        return 'short' if use_primitives else 'System.Int16'

    if type_ == 'uint16':
        return 'ushort' if use_primitives else 'System.UInt16'

    if type_ == 'int32':
        return 'int' if use_primitives else 'System.Int32'

    if type_ == 'uint32':
        return 'uint' if use_primitives else 'System.UInt32'

    if type_ == 'int64':
        return 'long' if use_primitives else 'System.Int64'

    if type_ == 'uint64':
        return 'ulong' if use_primitives else 'System.UInt64'

    if type_ == 'string':
        return 'System.String'

    assert False, "unknown type '%s'" % type_


def get_dotnet_type(type_, use_primitives=True):
    if not isinstance(type_, BasicType):
        return type_.namespaced_name

    return get_builtin_dotnet_type(type_.typename, use_primitives=use_primitives)

def upperfirst(s):
    return s[0].capitalize() + s[1:]

def get_field_name(type_name, field_name):
    if upperfirst(field_name) == type_name:
        return "{0}_".format(type_name)
    else:
        return upperfirst(field_name)
