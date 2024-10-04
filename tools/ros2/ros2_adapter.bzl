load(":ros2_providers.bzl", "IdlAdapterAspectInfo", "Ros2InterfaceInfo")
load(":ros2_utils.bzl", "get_basename_stem")

visibility(["//tools/..."])

def _run_adapter(ctx, package_name, srcs):
    adapter_arguments = struct(
        package_name = package_name,
        non_idl_tuples = [":{}".format(src.path) for src in srcs],
    )

    adapter_arguments_file = ctx.actions.declare_file(
        "{}/rosidl_adapter__arguments__{}.json".format(package_name, package_name),
    )
    ctx.actions.write(adapter_arguments_file, adapter_arguments.to_json())
    adapter_map = ctx.actions.declare_file(
        "{}/rosidl_adapter_map.idls".format(package_name),
    )
    output_dir = adapter_map.dirname

    idl_files = []
    ddsidl_files = []
    idl_tuples = []
    for src in srcs:
        extension = src.extension
        stem = get_basename_stem(src)
        idl_files.append(ctx.actions.declare_file(
            "{}/{}/{}.idl".format(package_name, extension, stem),
        ))
        ddsidl_files.append(ctx.actions.declare_file(
            "{}/{}/{}_.idl".format(package_name, extension, stem),
        ))
        idl_tuples.append(
            "{}:{}/{}.idl".format(output_dir, extension, stem),
        )

    # Run rosidl_adapter to convert .msg -> .idl
    adapter_cmd_args = ctx.actions.args()
    adapter_cmd_args.add(package_name, format = "--package-name=%s")
    adapter_cmd_args.add(adapter_arguments_file, format = "--arguments-file=%s")
    adapter_cmd_args.add(output_dir, format = "--output-dir=%s")
    adapter_cmd_args.add(adapter_map, format = "--output-file=%s")

    ctx.actions.run(
        inputs = srcs + [adapter_arguments_file],
        outputs = [adapter_map] + idl_files,
        executable = ctx.executable._adapter,
        arguments = [adapter_cmd_args],
        mnemonic = "Ros2IdlAdapter",
        progress_message = "Generating IDL files for %{label}",
    )

    # Run rosidl_dds to convert Ros2 Foo.idl -> DDS Foo_.idl
    ddsidl_templates = ctx.attr._rosidl_generator_dds_idl_templates[DefaultInfo].files.to_list()
    ddsidl_arguments = struct(
        package_name = package_name,
        idl_tuples = idl_tuples,
        output_dir = output_dir,
        template_dir = str(ddsidl_templates[0].dirname),
        target_dependencies = [],
    )

    ddsidl_arguments_file = ctx.actions.declare_file(
        "{}/rosidl_generator_dds_idl__arguments__{}.json".format(package_name, package_name),
    )
    ctx.actions.write(ddsidl_arguments_file, ddsidl_arguments.to_json())
    ddsidl_cmd_args = ctx.actions.args()
    ddsidl_cmd_args.add(ddsidl_arguments_file.path, format = "--generator-arguments-file=%s")
    ddsidl_cmd_args.add("", format = "--additional-service-templates=%s")
    ddsidl_cmd_args.add("", format = "--subfolders=%s")
    # ddsidl_cmd_args.add(extension, format = "--extension=%s")

    ctx.actions.run(
        inputs = idl_files + ddsidl_templates + [ddsidl_arguments_file],
        outputs = ddsidl_files,
        executable = ctx.executable._rosidl_generator_dds_idl,
        arguments = [ddsidl_cmd_args],
        mnemonic = "Ros2DdsIdlGenerator",
        progress_message = "Generating DDS_IDL files for %{label}",
    )

    return idl_files, ddsidl_files, idl_tuples

def _get_transitive_idl_deps(deps):
    direct_idl, direct_ddsidl = [], []
    for dep in deps:
        if IdlAdapterAspectInfo in dep:
            direct_idl.extend(dep[IdlAdapterAspectInfo].idl_files)
            direct_ddsidl.extend(dep[IdlAdapterAspectInfo].ddsidl_files)

    return (
        depset(
            direct = direct_idl,
            transitive = [
                dep[IdlAdapterAspectInfo].idl_deps
                for dep in deps
                if IdlAdapterAspectInfo in dep
            ],
        ),
        depset(
            direct = direct_ddsidl,
            transitive = [
                dep[IdlAdapterAspectInfo].ddsidl_deps
                for dep in deps
                if IdlAdapterAspectInfo in dep
            ],
        ),
    )

def _idl_adapter_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    idl_files, ddsidl_files, idl_tuples = _run_adapter(ctx, package_name, srcs)
    idl_deps, ddsidl_deps = _get_transitive_idl_deps(ctx.rule.attr.deps)

    return [
        IdlAdapterAspectInfo(
            idl_files = idl_files,
            ddsidl_files = ddsidl_files,
            idl_tuples = idl_tuples,
            idl_deps = idl_deps,
            ddsidl_deps = ddsidl_deps,
        ),
    ]

idl_adapter_aspect = aspect(
    implementation = _idl_adapter_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_adapter": attr.label(
            default = Label("@ros2//rosidl:rosidl_adapter_app"),
            executable = True,
            cfg = "exec",
        ),
        "_rosidl_generator_dds_idl": attr.label(
            default = Label("@ros2//rosidl_dds:rosidl_generator_dds_idl_app"),
            executable = True,
            cfg = "exec",
        ),
        "_rosidl_generator_dds_idl_templates": attr.label(
            default = Label("@ros2//rosidl_dds:rosidl_generator_dds_idl_templates"),
        ),
    },
    required_providers = [[Ros2InterfaceInfo]],
    provides = [IdlAdapterAspectInfo],
)
