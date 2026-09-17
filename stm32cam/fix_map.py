Import("env")

# 过滤掉 SCons / PlatformIO 自动注入的带中文路径的 -Wl,-Map 参数
def remove_map_flags(target, source, env):
    new_flags = []
    for f in env.get("LINK_FLAGS", []):
        if "-Map" not in str(f):
            new_flags.append(f)
    env["LINK_FLAGS"] = new_flags

    new_linkflags = []
    for f in env.get("LINKFLAGS", []):
        if "-Map" not in str(f):
            new_linkflags.append(f)
    env["LINKFLAGS"] = new_linkflags

env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", remove_map_flags)
