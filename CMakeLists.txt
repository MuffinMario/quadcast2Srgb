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
find_package(hidapi REQUIRED)
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

# binary
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
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

# configs
install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/config/"
    DESTINATION "etc/quadcast2srgb"
)

# optional systemd svc
if(USE_SYSTEMD)
    install(FILES "${CMAKE_SOURCE_DIR}/resources/systemd/quadcast2srgb.service"
        DESTINATION "lib/systemd/user"
    )
endif()

# udev rule for device access without sudo
install(FILES "${CMAKE_SOURCE_DIR}/resources/udev/99-quadcast2srgb.rules"
    DESTINATION "lib/udev/rules.d"
)