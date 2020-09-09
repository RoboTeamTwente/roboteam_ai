# Find programs and assign the boolean to the variables, find_program(variable binary)
find_program(CCACHE ccache)
find_program(GCC g++)
find_program(CLANG clang++)

# Some ccache logic
if (CLANG)
    set(CXX_COMPILER clang++)           # Set the compiler
    set(CXX_COMPILER_PATH ${CLANG})     # Set the path of the compiler, for if ccache not is used
elseif (GCC)
    set(CXX_COMPILER g++)               # Set the compiler
    set(CXX_COMPILER_PATH ${GCC})       # Set the path of the compiler, for if ccache not is used
endif()

if(CCACHE)
    set(CMAKE_CXX_COMPILER "/lib/ccache/${CXX_COMPILER}")
else()
    set(CMAKE_CXX_COMPILER "${CXX_COMPILER_PATH}")
endif()
# end of ccache logic