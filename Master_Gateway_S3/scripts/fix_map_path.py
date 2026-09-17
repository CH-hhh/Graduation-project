Import("env")

# The bundled Xtensa linker cannot create a map file through a Windows path
# containing non-ASCII characters. Keep PlatformIO's map output, but point it
# at an ASCII-only path relative to the project working directory.
link_flags = env.get("LINKFLAGS", [])
for index, flag in enumerate(link_flags):
    if isinstance(flag, str) and flag.startswith("-Wl,-Map="):
        link_flags[index] = "-Wl,-Map=firmware.map"
env.Replace(LINKFLAGS=link_flags)
