# Set the library name:
set(libname "evaDataHandler")

# List all source files:
set(sourceFiles
"src/evaDataHandler.cpp" 
"src/dataInput/dataStructures/location.cpp" 
"src/dataInput/dataStructures/locations.cpp" 
"src/dataInput/dataInput.cpp" 
"src/dataInput/dataStructures/config.cpp" 
"src/dataInput/dataStructures/chargers.cpp" 
"src/dataInput/dataStructures/trips.cpp" 
"src/dataInput/dataStructures/vehicles.cpp" 
"src/dataInput/dataStructures/maintenances.cpp" 
"src/scheduleGraph/scheduleGraph.cpp" 
"src/dataOutput/dataOutput.cpp" )

# List all header files:
set(headerFiles
"incl/evaDataHandler.h" 
"incl/dataInput/dataInput.h"
"incl/dataInput/dataStructures/charger.h"
"incl/dataInput/dataStructures/chargers.h"
"incl/dataInput/dataStructures/config.h"
"incl/dataInput/dataStructures/location.h"
"incl/dataInput/dataStructures/locations.h"
"incl/dataInput/dataStructures/maintenance.h"
"incl/dataInput/dataStructures/maintenances.h"
"incl/dataInput/dataStructures/trip.h"
"incl/dataInput/dataStructures/trips.h"
"incl/dataInput/dataStructures/vehicle.h"
"incl/dataInput/dataStructures/vehicles.h"
"incl/scheduleGraph/scheduleGraph.h"
"incl/scheduleGraph/scheduleArc.h"
"incl/scheduleGraph/scheduleNode.h"
"incl/dataStats/stats.h"
"incl/dataOutput/dataOutput.h")

# Add the library, including all source files:
add_library(${libname} STATIC ${sourceFiles} ${headerFiles})

# Link the libraries
target_link_libraries(${libname} PUBLIC evaGlobal Boost::graph)
target_link_libraries(${libname} PRIVATE Boost::program_options Boost::filesystem)

target_include_directories(${libname} 
PRIVATE 
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(${libname} 
PUBLIC 
	${Boost_INCLUDE_DIRS}
	"incl"
)