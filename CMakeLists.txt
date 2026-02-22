cmake_minimum_required(VERSION 3.16)

project(myproject
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# library requirements
find_package(hidapi REQUIRED)
#find_package(flatbuffers REQUIRED)

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
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    hidapi::hidapi
    #flatbuffers::flatbuffers
)
