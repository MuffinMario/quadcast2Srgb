cmake_minimum_required(VERSION 3.16)

project(qc2srgb
    VERSION 0.4.0
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

# PortAudio + FFTW3 (audio capture + spectrum analysis)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED IMPORTED_TARGET portaudio-2.0)
pkg_check_modules(FFTW3    REQUIRED IMPORTED_TARGET fftw3)

# systemd integration (opt-in via -DUSE_SYSTEMD=ON)
option(USE_SYSTEMD "Enable systemd integration" OFF)

# GLSL shader display (opt-in via -DUSE_GLSL=ON)
# Requires: libEGL and OpenGL ES 3 (e.g. Mesa's libGLESv2)
option(USE_GLSL "Enable GLSL shader display via EGL + OpenGL ES 3" ON)

if(USE_SYSTEMD)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SYSTEMD libsystemd)
endif()

if(USE_GLSL)
    find_package(PkgConfig REQUIRED)
    # libEGL
    find_package(OpenGL REQUIRED COMPONENTS EGL)
    # OpenGL ES 3 (glesv2 provides the ES 2+3 entry points on Mesa)
    pkg_check_modules(GLESV2 REQUIRED glesv2)
endif()

# Clang-Tidy integration (opt-out via -DENABLE_CLANG_TIDY=OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" ON)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_PATH "clang-tidy" REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY
        "clang-tidy"
        -checks=readability-identifier-naming # prevent "no checks enabled"
        --warnings-as-errors=readability-identifier-naming
    )
endif()

# Common sources: everything except Main*.cpp and the HID communicator layer. (not recommended to just wildcard, tbh. fine in our case...)
file(GLOB_RECURSE COMMON_SOURCES
    "${CMAKE_SOURCE_DIR}/src/audio/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/config/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/display/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/util/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/video/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/Globals.cpp"
)

# Hardware (HID) executable sources
# Main.cpp
set(HW_SOURCES
    ${COMMON_SOURCES}
    "${CMAKE_SOURCE_DIR}/src/Main.cpp"
)
# hid and communicator stuff, also exclusive to the hardware exec
file(GLOB_RECURSE HW_HID_SOURCES
    "${CMAKE_SOURCE_DIR}/src/communicator/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/hid/*.cpp"
)
list(APPEND HW_SOURCES ${HW_HID_SOURCES})

# Preview (window) executable sources
set(PREVIEW_SOURCES
    ${COMMON_SOURCES}
    "${CMAKE_SOURCE_DIR}/src/MainPreview.cpp"
)

# Dear ImGui — added directly per getting-started guide.
# .clang-tidy lives in src/ so external/ files are unaffected.
file(GLOB IMGUI_SOURCES
    "${CMAKE_SOURCE_DIR}/external/imgui/imgui.cpp"
    "${CMAKE_SOURCE_DIR}/external/imgui/imgui_draw.cpp"
    "${CMAKE_SOURCE_DIR}/external/imgui/imgui_widgets.cpp"
    "${CMAKE_SOURCE_DIR}/external/imgui/imgui_tables.cpp"
    "${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_sdl2.cpp"
    "${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3.cpp"
)
list(APPEND PREVIEW_SOURCES ${IMGUI_SOURCES})

add_executable(${PROJECT_NAME} ${HW_SOURCES})

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic

    $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    hidapi::hidapi
    PkgConfig::PORTAUDIO
    PkgConfig::FFTW3
)


if(USE_SYSTEMD)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SYSTEMD_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SYSTEMD)
endif()

if(USE_GLSL)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::EGL ${GLESV2_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${GLESV2_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GLSL)
endif()

# ── Preview executable (qc2srgb-preview) ──────────────────────────────────
find_package(SDL2 REQUIRED)

add_executable(${PROJECT_NAME}-preview ${PREVIEW_SOURCES})

target_compile_options(${PROJECT_NAME}-preview PRIVATE
    -Wall
    -Wextra
    -Wpedantic

    $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
)
target_compile_definitions(${PROJECT_NAME}-preview PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
    PREVIEW_WINDOW_BINARY
    IMGUI_IMPL_OPENGL_ES3
)
target_link_libraries(${PROJECT_NAME}-preview PRIVATE
    PkgConfig::PORTAUDIO
    PkgConfig::FFTW3
    SDL2::SDL2
)

# Preview doesn't use the HID/communicator layer or systemd.
# USE_GLSL is still supported for the GLSL shader display.

if(USE_GLSL)
    target_link_libraries(${PROJECT_NAME}-preview PRIVATE OpenGL::EGL ${GLESV2_LIBRARIES})
    target_include_directories(${PROJECT_NAME}-preview PRIVATE ${GLESV2_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME}-preview PRIVATE USE_GLSL)
endif()

# ImGui include paths
target_include_directories(${PROJECT_NAME}-preview PRIVATE
    "${CMAKE_SOURCE_DIR}/external/imgui"
    "${CMAKE_SOURCE_DIR}/external/imgui/backends"
)

# Copy config to preview output directory as well
add_custom_command(
    TARGET ${PROJECT_NAME}-preview POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/resources/config/config.toml"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}-preview>/config.toml"
    COMMENT "Copying config.toml to preview output directory"
)

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

if(CMAKE_INSTALL_PREFIX MATCHES "/usr$")
    # keep the path before /usr, important when deploying to a prefix, usual when packaging
    string(REGEX REPLACE "/usr$" "/etc" _sysconfdir "${CMAKE_INSTALL_PREFIX}")
    set(CMAKE_INSTALL_SYSCONFDIR "${_sysconfdir}" CACHE PATH "System configuration directory" FORCE)
endif()

# Shader install directory (used to substitute paths in config templates)
set(QUADCAST2SRGB_SHADER_DIR "${CMAKE_INSTALL_FULL_DATADIR}/quadcast2srgb/shaders")

# Generate config files that embed the resolved shader install path
configure_file(
    "${CMAKE_SOURCE_DIR}/resources/config/config_shader.toml.in"
    "${CMAKE_BINARY_DIR}/config/config_shader.toml"
    @ONLY
)
configure_file(
    "${CMAKE_SOURCE_DIR}/resources/config/example_config_alternate_full.toml.in"
    "${CMAKE_BINARY_DIR}/config/example_config_alternate_full.toml"
    @ONLY
)

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
# .toml.in sources are excluded; the configure_file()-generated versions are installed instead
install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/config/"
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/quadcast2srgb"
    REGEX "\\.toml\\.in$" EXCLUDE
)
install(FILES
    "${CMAKE_BINARY_DIR}/config/config_shader.toml"
    "${CMAKE_BINARY_DIR}/config/example_config_alternate_full.toml"
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/quadcast2srgb"
)

# GLSL shaders (directory may be absent in older/partial source tarballs — skip gracefully)
if(USE_GLSL AND EXISTS "${CMAKE_SOURCE_DIR}/resources/shader")
    install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/shader/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/quadcast2srgb/shaders"
    )
endif()

# optional systemd svc
if(USE_SYSTEMD)
    # Substitute the real binary and config paths into the unit file at configure time
    configure_file(
        "${CMAKE_SOURCE_DIR}/resources/systemd/quadcast2srgb.service.in"
        "${CMAKE_BINARY_DIR}/quadcast2srgb.service"
        @ONLY
    )
    install(FILES "${CMAKE_BINARY_DIR}/quadcast2srgb.service"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/systemd/user" 
        # dont use libdir... multiarch systems have different libdirs which are not appropriate for systemd and udev
    )
endif()

# udev rule for device access without sudo
install(FILES "${CMAKE_SOURCE_DIR}/resources/udev/99-quadcast2srgb.rules"
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/udev/rules.d"
)

# license
install(FILES "${CMAKE_SOURCE_DIR}/LICENSE"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/licenses/quadcast2srgb"
)