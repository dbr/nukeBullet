cmake_minimum_required(VERSION 2.8)

# Pick up share/cmake/ modules
# TODO: Append instead of overriding
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/share/cmake)

# If CMAKE_INSTALL_EXEC_PREFIX is not specified, install binaries
# directly into the regular install prefix
if(NOT CMAKE_INSTALL_EXEC_PREFIX)
    message("Exec prefix not specified, defaulting to ${CMAKE_INSTALL_PREFIX}")
    set(CMAKE_INSTALL_EXEC_PREFIX ${CMAKE_INSTALL_PREFIX})
endif()

# Locate Nuke or error
find_package(Nuke)
if(NUKE_FOUND)
    message("Build Nuke plugins against ${Nuke_LIBRARY_DIR}")
else()
    message(FATAL_ERROR "Cannot locate Nuke. Add the flag -D NUKE_INSTALL_PATH=... or set the NDK_PATH environment variable")
endif()


# Include Nuke headers
include_directories(
    ${Nuke_INCLUDE_DIR}
    /Users/dbr/Downloads/bullet-2.79/build/dist/include/bullet
)

# Add plugin as shared library (.so on linux, .dylib on OS X)
add_library(BulletSolver SHARED
    src/BulletSolver.cpp
)

message(${Nuke_LIBRARIES})
# Link against Nuke's libraries
target_link_libraries(BulletSolver
    ${Nuke_LIBRARIES}
    /Users/dbr/Downloads/bullet-2.79/build/dist/lib/libBulletDynamics.a
    /Users/dbr/Downloads/bullet-2.79/build/dist/lib/libBulletCollision.a
    /Users/dbr/Downloads/bullet-2.79/build/dist/lib/libLinearMath.a
    /Users/dbr/Downloads/bullet-2.79/build/dist/lib/libBulletSoftBody.a
)

# Prevent lib prefix on filesnames
set_target_properties(BulletSolver
    PROPERTIES
    PREFIX ""
)

# Once built, copy the library into install location (e.g "executable-prefix/lib/nuke/6.2")
install(TARGETS BulletSolver
        DESTINATION ${CMAKE_INSTALL_EXEC_PREFIX}/lib/nuke${Nuke_DDImageVersion})
