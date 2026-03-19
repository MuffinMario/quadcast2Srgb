cmake_minimum_required(VERSION 3.16)

project(qc2srgb
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generates compile_commands.json for clangd / VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# library requirements
# hidapi: prefer the CMake config (Arch ships one); fall back to pkg-config (Debian/Ubuntu)
find_package(hidapi QUIET)
if(hidapi_FOUND)
    message(STATUS "hidapi found via CMake config")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(hidapi REQUIRED IMPORTED_TARGET hidapi-hidraw)
    add_library(hidapi::hidapi ALIAS PkgConfig::hidapi)
    message(STATUS "hidapi found via pkg-config (hidapi-hidraw)")
endif()
#find_package(flatbuffers REQUIRED)

# systemd integration (opt-in via -DUSE_SYSTEMD=ON)
option(USE_SYSTEMD "Enable systemd integration" OFF)

if(USE_SYSTEMD)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SYSTEMD libsystemd)
endif()

# Clang-Tidy integration (opt-out via -DENABLE_CLANG_TIDY=OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" ON)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_PATH "clang-tidy" REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY
        "clang-tidy"
        --warnings-as-errors=readability-identifier-naming
    )
endif()

# Generally not advisable, recommended instead is to list up every source file explicitly!
file(GLOB_RECURSE SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic

    $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    hidapi::hidapi
)


if(USE_SYSTEMD)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SYSTEMD_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SYSTEMD)
endif()

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/resources/config/config.toml"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config.toml"
    COMMENT "Copying config.toml to output directory"
)


# install task:

# set up CORRECT install dirs to deploy during install
# at this point why even use this variable, but... changes sysconfdir to /etc if it is /usr, 
# to avoid installing config files to /usr/etc
include(GNUInstallDirs)
#print the install dirs
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_BINDIR: ${CMAKE_INSTALL_BINDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_SYSCONFDIR: ${CMAKE_INSTALL_SYSCONFDIR}")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")

if(CMAKE_INSTALL_PREFIX MATCHES "/usr$")
    # keep the path before /usr, important when deploying to a prefix, usual when packaging
    string(REGEX REPLACE "/usr$" "/etc" _sysconfdir "${CMAKE_INSTALL_PREFIX}")
    set(CMAKE_INSTALL_SYSCONFDIR "${_sysconfdir}" CACHE PATH "System configuration directory" FORCE)
endif()
message(STATUS "CMAKE_INSTALL_SYSCONFDIR (post): ${CMAKE_INSTALL_SYSCONFDIR}")

# binary
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
    @ONLY
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake
    COMMENT "Uninstall files listed in install_manifest.txt"
)

# install default config skeleton into sysconf (packagers should move to /etc)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/config/"
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/quadcast2srgb"
)

# optional systemd svc
if(USE_SYSTEMD)
    # Substitute the real binary and config paths into the unit file at configure time
    configure_file(
        "${CMAKE_SOURCE_DIR}/resources/systemd/quadcast2srgb.service.in"
        "${CMAKE_BINARY_DIR}/quadcast2srgb.service"
        @ONLY
    )
    install(FILES "${CMAKE_BINARY_DIR}/quadcast2srgb.service"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/systemd/user"
    )
endif()

# udev rule for device access without sudo
install(FILES "${CMAKE_SOURCE_DIR}/resources/udev/99-quadcast2srgb.rules"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/udev/rules.d"
)