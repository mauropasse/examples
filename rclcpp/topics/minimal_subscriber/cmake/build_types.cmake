# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable debug symbols
function(debug_symbols)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g" PARENT_SCOPE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g" PARENT_SCOPE)
endfunction()

# Warnings are errors
function(warnings_are_errors)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -fdiagnostics-color=always" PARENT_SCOPE)
endfunction()

# Position independent code
function(position_independent_code)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" PARENT_SCOPE)
endfunction()

# Optimization
function(optimization)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2" PARENT_SCOPE)
endfunction()

# Strict compilation
function(strict_compilation)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstrict-overflow -fdata-sections -ffunction-sections -fstack-protector" PARENT_SCOPE)
endfunction()

# Apply compilation flags
warnings_are_errors()
position_independent_code()
optimization()
strict_compilation()

# Check if compilation should be done without debug symbols
if(NOT DEFINED $ENV{NO_DEBUG_SYMBOLS} OR $ENV{NO_DEBUG_SYMBOLS} STREQUAL "FALSE")
  debug_symbols()
endif()
