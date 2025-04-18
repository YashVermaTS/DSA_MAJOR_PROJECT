cmake_minimum_required(VERSION 3.10)
project(DSA_major_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add option to specify SFML path if needed
option(SFML_DIR "Path to SFML installation" "")

# Find SFML with more robust error handling
find_package(SFML 2.5 COMPONENTS graphics window system)
if(NOT SFML_FOUND)
    message(WARNING "SFML not found. Trying to build without SFML visualization.")
    add_definitions(-DNO_SFML_VISUALIZATION)
else()
    message(STATUS "SFML found. Building with visualization support.")
endif()

# Detect source files automatically
file(GLOB_RECURSE SOURCE_FILES 
     "${CMAKE_SOURCE_DIR}/src/*.cpp"
     "${CMAKE_SOURCE_DIR}/include/*.cpp"
)

# Print detected files for debugging
message(STATUS "Detected header files: ${HEADER_FILES}")
message(STATUS "Detected source files: ${SOURCE_FILES}")

# Include directories - add current directory and any include directory
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include)

# Create executable with all source files
add_executable(pacman_maze_solver ${SOURCE_FILES})

# Link libraries conditionally
if(SFML_FOUND)
    target_link_libraries(pacman_maze_solver sfml-graphics sfml-window sfml-system)
endif()

# Add threading support if needed
find_package(Threads)
if(Threads_FOUND)
    target_link_libraries(pacman_maze_solver Threads::Threads)
endif()

# Install target
install(TARGETS pacman_maze_solver DESTINATION bin)

# Print configuration summary
message(STATUS "Configuration Summary:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  SFML Found: ${SFML_FOUND}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")

