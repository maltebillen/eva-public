# Set the library name:
set(libname "evaGlobal")

# List all source files:
set(sourceFiles
	"src/evaConstants.cpp")

set(headerFiles
	"incl/evaConstants.h"
	"incl/evaExceptions.h")

# Add the library, including all source files:
add_library(${libname} STATIC ${sourceFiles} ${headerFiles})

target_include_directories(${libname} 
PUBLIC 
	"incl"
)