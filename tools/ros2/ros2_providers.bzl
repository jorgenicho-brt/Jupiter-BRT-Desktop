visibility(["public"])

Ros2InterfaceInfo = provider(
    "Provides info for interface code generation.",
    fields = [
        "info",
        "deps",
    ],
)

IdlAdapterAspectInfo = provider(
    "Provider for IDL codegen from Ros2 messages",
    fields = [
        "idl_files",
        "ddsidl_files",
        "idl_tuples",
        "idl_deps",
        "ddsidl_deps",
    ],
)

CGeneratorAspectInfo = provider(
    "Provider for C codegen for Ros2 messages",
    fields = [
        "cc_info",
    ],
)

CppGeneratorAspectInfo = provider(
    "Provider for C++ codegen for Ros2 messages",
    fields = [
        "cc_info",
        "typesupport_sos",
    ],
)

PyGeneratorAspectInfo = provider(
    "Provider for Python codegen for Ros2 messages",
    fields = [
        "cc_info",
        "direct_sources",
        "dynamic_libraries",
        "transitive_sources",
        "imports",
    ],
)
