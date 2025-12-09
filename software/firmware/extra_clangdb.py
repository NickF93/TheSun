# extra_clangdb.py
# This script configures PlatformIO to generate a compile_commands.json
# suitable for clangd static analysis.

Import("env")

# Save compile_commands.json inside the build directory for the active env
env.Replace(
    COMPILATIONDB_PATH="$PROJECT_DIR/compile_commands.json"
)

# Do NOT enable COMPILATIONDB_INCLUDE_TOOLCHAIN here.
# For AVR we only need framework/project includes, not host system headers.
