@#######################################################################
@# Included from rosidl_generator_cs/resource/idl.cs.em
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - message (IdlMessage, with structure containing names, types and members)
@#######################################################################
@{
from rosidl_generator_cs import escape_string
from rosidl_generator_cs import escape_wstring
from rosidl_generator_cs import msg_type_to_cs
from rosidl_generator_cs import MSG_TYPE_TO_CS
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BOOLEAN_TYPE
from rosidl_parser.definition import CHARACTER_TYPES
from rosidl_parser.definition import INTEGER_TYPES
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import OCTET_TYPE
from rosidl_parser.definition import UNSIGNED_INTEGER_TYPES

}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@# Handle namespaces
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{

@[end for]@
// message class
public class @(message.structure.namespaced_type.name) : IRclcsMessage
{
  private IntPtr handle;
  private bool disposed;
  private bool isTopLevelMsg;

  // constant declarations
@[for constant in message.constants]@
@[ if isinstance(constant.type, AbstractString)]@
  public static const @(MSG_TYPE_TO_CS['string']) @(constant.name) = "@(escape_string(constant.value))";
@[ elif isinstance(constant.type, AbstractWString)]@
  public static const @(MSG_TYPE_TO_CS['wstring']) @(constant.name) = "@(escape_wstring(constant.value))";
@[ else]@
  public static const @(MSG_TYPE_TO_CS[constant.type.typename]) @(constant.name) =
@[  if isinstance(constant.type, BasicType) and constant.type.typename in (*INTEGER_TYPES, *CHARACTER_TYPES, BOOLEAN_TYPE, OCTET_TYPE)]@
    @(int(constant.value))@
@[  else]@
    @(constant.value);
@[  end if]@
@[ end if]@
@[end for]@

  // members
@[for member in message.structure.members]@
@[  if isinstance(member.type.value_type, BasicType)]@
  using _@(member.name)_type =
  @(msg_type_to_cs(member.type));
  public _@(member.name)_type @(member.name) { get; set; }
@[  end if]@
@[end for]@

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeGetTypeSupportType();
  private static NativeGetTypeSupportType native_get_typesupport = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeCreateNativeMessageType();
  private static NativeCreateNativeMessageType native_create_native_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeDestroyNativeMessageType(IntPtr messageHandle);
  private static NativeDestroyNativeMessageType native_destroy_native_message = null;

@[for member in @(message.structure.members)]@
@[  if isinstance(member.type.value_type, BasicType)]@
@[    if isinstance(member.type.value_type, AbstractString)]@
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadField@(get_field_name(member.type, member.name))Type(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteField@(get_field_name(member.type, member.name))Type(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);
@[    else]@
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate @(msg_type_to_cs(member.type)) NativeReadField@(get_field_name(member.type, member.name))Type(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteField@(get_field_name(member.type, member.name))Type(
    IntPtr messageHandle, @(msg_type_to_cs(member.type)) value);
@[    end if]@
  private static NativeReadField@(get_field_name(member.type, member.name))Type native_read_field_@(member.name) = null;
  private static NativeWriteField@(get_field_name(member.type, member.name))Type native_write_field_@(member.name) = null;
@[  end if]@
@[end for]@

  static @(message.structure.namespaced_type.name)()
  {
    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("@(package_name)_@(message.structure.namespaced_type.name)__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_get_typesupport");
    @(message.structure.namespaced_type.name).native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_create_native_message");
    @(message.structure.namespaced_type.name).native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_destroy_native_message");
    @(message.structure.namespaced_type.name).native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

@[for member in @(message.structure.members)]@
@[  if isinstance(member.type.value_type, BasicType)]@
    IntPtr native_read_field_@(member.name)_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "native_read_field_@(member.name)");
    @(message.structure.namespaced_type.name).native_read_field_@(member.name) =
      (NativeReadField@(get_field_name(member.type, member.name))Type)Marshal.GetDelegateForFunctionPointer(
      native_read_field_@(member.name)_ptr, typeof(NativeReadField@(get_field_name(member.type, member.name))Type));

    IntPtr native_write_field_@(member.name)_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "native_write_field_@(member.name)");
    @(message.structure.namespaced_type.name).native_write_field_@(member.name) =
      (NativeWriteField@(get_field_name(member.type, member.name))Type)Marshal.GetDelegateForFunctionPointer(
      native_write_field_@(member.name)_ptr, typeof(NativeWriteField@(get_field_name(member.type, member.name))Type));
@[  end if]@
@[end for]@
  }

  public static IntPtr GetTypeSupport()
  {
    return native_get_typesupport();
  }

  public @(message.structure.namespaced_type.name)()
  {
    isTopLevelMsg = true;
    handle = native_create_native_message();
    SetNestedHandles();
  }

  // internal constructor for nested types
  internal @(message.structure.namespaced_type.name)(IntPtr handle)
  {
    this.handle = handle;
    SetNestedHandles();
  }

  //TODO (adamdbrw): bad design. One has to call the constructor, extract the handle and modify it outside with rcl_take
  public void ReadNativeMessage()
  {
@[for member in @(message.structure.members)]@
@[  if isinstance(member.type.value_type, BasicType)]@
@[    if isinstance(member.type.value_type, AbstractString)]@
    {
      IntPtr pStr = native_read_field_@(member.name)(handle);
      @(get_field_name(member.type, member.name)) = Marshal.PtrToStringAnsi(pStr);
    }
@[    else]@
    @(get_field_name(member.type, member.name)) = native_read_field_@(member.name)(handle);
@[    end if]@
@[  end if]@
@[end for]@
  }

  public void WriteNativeMessage()
  {
@[for member in @(message.structure.members)]@
@[  if isinstance(member.type.value_type, BasicType)]@
    native_write_field_@(member.name)(messageHandle, @(get_field_name(member.type, member.name)));
@[  else]@
@[  end if]@
@[end for]@
  }

  public void Dispose()
  {
    if(!disposed)
    {
      if(isTopLevelMsg)
      {
        handle = native_destroy_native_message(handle);
        disposed = true;
      }
    }
  }

  ~@(message.structure.namespaced_type.name)()
  {
    Dispose();
  }

  // Handle
  public IntPtr Handle
  {
    get
    {
      return handle;
    }
  }
};  // class @(message.structure.namespaced_type.name)_
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@# close namespaces
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@[for ns in reversed(message.structure.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
