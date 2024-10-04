visibility(["public"])

def get_basename_stem(path):
    return path.basename[:-len(path.extension) - 1]

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

def _to_snake_case(not_snake_case):
    """Converts camel-case to snake-case.
    Based on convert_camel_case_to_lower_case_underscore from rosidl_cmake.
    Unfortunately regex doesn't exist in Bazel.
    Args:
      not_snake_case: a camel-case string.
    Returns:
      A snake-case string.
    """
    result = ""
    not_snake_case_padded = " " + not_snake_case + " "
    for i in range(len(not_snake_case)):
        prev_char, char, next_char = not_snake_case_padded[i:i + 3].elems()
        if char.isupper() and next_char.islower() and prev_char != " ":
            # Insert an underscore before any upper case letter which is not
            # followed by another upper case letter.
            result += "_"
        elif char.isupper() and (prev_char.islower() or prev_char.isdigit()):
            # Insert an underscore before any upper case letter which is
            # preseded by a lower case letter or number.
            result += "_"
        result += char.lower()

    return result

def run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        generator,
        generator_templates,
        output_mapping,
        visibility_control_template = None,
        extra_generator_args = None,
        extra_generated_outputs = None,
        mnemonic = None,
        progress_message = None):
    generator_templates = generator_templates[DefaultInfo].files.to_list()

    generator_arguments_file = ctx.actions.declare_file(
        "{}/{}__arguments.json".format(package_name, generator.basename),
    )
    output_dir = generator_arguments_file.dirname
    generator_arguments = struct(
        package_name = package_name,
        idl_tuples = adapter.idl_tuples,
        output_dir = output_dir,
        template_dir = generator_templates[0].dirname,
        target_dependencies = [],
    )
    ctx.actions.write(generator_arguments_file, generator_arguments.to_json())

    generator_cmd_args = ctx.actions.args()
    generator_cmd_args.add(
        generator_arguments_file.path,
        format = "--generator-arguments-file=%s",
    )
    if extra_generator_args:
        for arg in extra_generator_args:
            generator_cmd_args.add(arg)

    generator_outputs = []
    for src in srcs:
        extension = src.extension
        stem = get_basename_stem(src)
        snake_case_stem = _to_snake_case(stem)
        for t in output_mapping:
            relative_file = "{}/{}/{}".format(
                package_name,
                extension,
                t % snake_case_stem,
            )
            generator_outputs.append(ctx.actions.declare_file(relative_file))

    extra_generated_outputs = extra_generated_outputs or []
    for extra_output in extra_generated_outputs:
        relative_file = "{}/{}".format(package_name, extra_output)
        generator_outputs.append(ctx.actions.declare_file(relative_file))

    ctx.actions.run(
        inputs = adapter.idl_files + generator_templates + [generator_arguments_file],
        outputs = generator_outputs,
        executable = generator,
        arguments = [generator_cmd_args],
        mnemonic = mnemonic,
        progress_message = progress_message,
    )

    if visibility_control_template:
        visibility_control_basename = get_basename_stem(visibility_control_template)
        relative_file = "{}/msg/{}".format(
            package_name,
            visibility_control_basename,
        )
        visibility_control_h = ctx.actions.declare_file(relative_file)
        generator_outputs.append(visibility_control_h)
        ctx.actions.expand_template(
            template = visibility_control_template,
            output = visibility_control_h,
            substitutions = {
                "@PROJECT_NAME@": package_name,
                "@PROJECT_NAME_UPPER@": package_name.upper(),
            },
        )

    cc_include_dir = _get_parent_dir(output_dir)
    return generator_outputs, cc_include_dir
