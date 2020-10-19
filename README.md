# leeward

**L**idar **e**quation **e**ngine **w**ith **a**lready **r**acked **d**erivatives.

Includes a Rust API and a (limited) C API.

## C API usage

The two files you'll need to use the C API are built into `target/<profile>`, where `<profile>` is either `debug` or `release`.
Here's an example CMake [find module](https://cmake.org/cmake/help/v3.14/manual/cmake-developer.7.html#find-modules):

```cmake
if(NOT DEFINED Leeward_DIR)
    set(Leeward_DIR "" CACHE PATH "Path to the build directory for the desired profile (e.g. /path/to/leeward/ttarget/release)")
endif()

find_path(Leeward_INCLUDE_DIR
    NAMES leeward.h
    PATHS ${Leeward_DIR}
)
find_library(Leeward_LIBRARY
    NAMES libleeward.a
    PATHS ${Leeward_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Leeward
    FOUND_VAR Leeward_FOUND
    REQUIRED_VARS
        Leeward_INCLUDE_DIR
        Leeward_LIBRARY
)

if(Leeward_FOUND AND NOT TARGET Leeward::Leeward)
    add_library(Leeward::Leeward UNKNOWN IMPORTED)
    set_target_properties(Leeward::Leeward PROPERTIES
        IMPORTED_LOCATION "${Leeward_LIBRARY}" 
        INTERFACE_INCLUDE_DIRECTORIES "${Leeward_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(Leeward_INCLUDE_DIR)
mark_as_advanced(Leeward_LIBRARY)
```
