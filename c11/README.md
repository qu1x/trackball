# C11 Implementation

[Documentation](https://doc.qu1x.dev/trackball/c11)

## Build

```sh
# Create build directory.
meson setup --buildtype release release/
# Compile library.
meson compile -C release/
# Test example.
meson test -C release/
# Install library.
meson install -C release/
# Create documentation.
doxygen
# Open documentation.
xdg-open doc/html/index.html
# Clean generated files.
git clean -Xdf
```
