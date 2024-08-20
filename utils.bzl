def test_function():
    pass

def pyinit_file_genrule(name, out_path):
    native.genrule(
        name = name,
        outs = [out_path + "/__init__.py"],
        srcs = ["templates/__init__.py.template"],
        cmd = "cp $< $@"
    )

def _create_symlinks_impl(ctx):
    files = []
    for dep in ctx.attr.deps:
        files.extend(dep[DefaultInfo].files.to_list())

    links = []
    for file in files:
        syml = ctx.actions.declare_file(file.basename)
        ctx.actions.symlink(output = syml, target_file = file)
        links.append(syml)
        print("Added symlink " + file.basename + " to file " + file.path)

    return [
        DefaultInfo(
            runfiles = ctx.runfiles(files = links),
        ),
    ]

create_symlinks = rule(
    attrs = {
        "deps": attr.label_list(
            allow_empty = False,
            mandatory = True,
        ),
    },
    implementation = _create_symlinks_impl,
)