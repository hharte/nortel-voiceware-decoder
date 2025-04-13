# CMakeLists.txt for Nortel Millennium VoiceWare Decoder

cmake_minimum_required(VERSION 3.10)

project(NortelDecoder C)

# Set C standard to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF) # Use standard C99, not GNU extensions

# Define the executable using the new source file name and new executable name
# Ensure you rename the actual C file to nortel-voiceware-decoder.c
add_executable(nortel-voiceware-decoder nortel-voiceware-decoder.c)

# --- Git Version Information ---
# Find Git executable
find_package(Git QUIET)

if(GIT_FOUND)
    # Get Commit Hash (Short)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH_INTERNAL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET # Don't fail build if git command fails, use default
        RESULT_VARIABLE GIT_HASH_RESULT
    )
    if(NOT GIT_HASH_RESULT EQUAL 0)
        set(GIT_COMMIT_HASH_INTERNAL "error") # Indicate command failure
        message(WARNING "Failed to get Git commit hash.")
    endif()

    # Get Tag Name (or fallback using describe --always)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty=-modified
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG_NAME_INTERNAL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE GIT_TAG_RESULT
    )
    if(NOT GIT_TAG_RESULT EQUAL 0)
        set(GIT_TAG_NAME_INTERNAL "error") # Indicate command failure
        message(WARNING "Failed to get Git tag/describe.")
    endif()

else()
    # Git not found, use defaults
    set(GIT_COMMIT_HASH_INTERNAL "nogit")
    set(GIT_TAG_NAME_INTERNAL "nogit")
    message(STATUS "Git not found. Using default version info.")
endif()

# Pass Git info defines to the compiler
target_compile_definitions(nortel-voiceware-decoder PRIVATE
    "GIT_COMMIT_HASH=\"${GIT_COMMIT_HASH_INTERNAL}\""
    "GIT_TAG_NAME=\"${GIT_TAG_NAME_INTERNAL}\""
)

# Optionally print the values during configuration
message(STATUS "Git Commit Hash: ${GIT_COMMIT_HASH_INTERNAL}")
message(STATUS "Git Tag/Describe: ${GIT_TAG_NAME_INTERNAL}")
# --- End Git Version Information ---


# Link math library on POSIX systems (needed for some functions, though maybe not strictly here)
if(UNIX AND NOT APPLE)
    target_link_libraries(nortel-voiceware-decoder m)
endif()

# Optional: Add compiler flags for warnings (adjust as needed)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(nortel-voiceware-decoder PRIVATE -Wall -Wextra -pedantic)
elseif(MSVC)
    # MSVC specific flags (e.g., /W4 for high warning level)
    target_compile_options(nortel-voiceware-decoder PRIVATE /W3)
    # Add definitions already handled by #defines in code, but could be added here too
    # target_compile_definitions(nortel-voiceware-decoder PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Installation (optional)
# install(TARGETS nortel-voiceware-decoder DESTINATION bin)

# Display build info
message(STATUS "Building Nortel Voiceware Decoder")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}") # Debug, Release, etc.
