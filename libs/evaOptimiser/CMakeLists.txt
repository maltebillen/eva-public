# Set the library name:
set(libname "evaOptimiser")

# List all source files:
set(sourceFiles
"src/evaOptimiser.cpp"
"src/moderator/OptimisationInput.cpp"
"src/masterProblem/masterProblem.cpp"
"src/pricingProblem/pricingProblem.cpp"
"src/pricingProblem/timeSpace/timeSpaceNetwork.cpp"
 "src/moderator/moderator.cpp" 
 "src/pricingProblem/segmentBased/segments/segments.cpp"
 "src/pricingProblem/segmentBased/segments/subGraph/subGraph.cpp" 
 "src/pricingProblem/segmentBased/segments/segment.cpp" 
 "src/moderator/chargingStrategies.cpp"
"src/pricingProblem/segmentBased/connectionNetwork/segmentConnectionNetwork.cpp" 
"src/pricingProblem/segmentBased/centralisedNetwork/segmentCentralisedNetwork.cpp" )

# List all header files:
set(headerFiles
"incl/evaOptimiser.h" 
"incl/masterProblem/masterProblem.h"
"incl/moderator/branch.h"
"incl/moderator/chargingStrategies.h"
"incl/moderator/moderator.h"
"incl/moderator/OptimisationInput.h" 
"incl/moderator/SubScheduleNodes.h"
"incl/pricingProblem/segmentBased/segments/subGraph/subGraph.h"
"incl/pricingProblem/segmentBased/segments/subGraph/subGraphArcs.h"
"incl/pricingProblem/segmentBased/segments/subGraph/subGraphNodes.h"
"incl/pricingProblem/segmentBased/segments/segment.h"
"incl/pricingProblem/segmentBased/segments/segments.h"
"incl/pricingProblem/timeSpace/timeSpaceArcs.h"
"incl/pricingProblem/timeSpace/timeSpaceNetwork.h"
"incl/pricingProblem/timeSpace/timeSpaceNodes.h"
"incl/pricingProblem/pricingProblem.h"
"incl/pricingProblem/segmentBased/connectionNetwork/segmentConnectionNetworkArcs.h"
"incl/pricingProblem/segmentBased/connectionNetwork/segmentConnectionNetwork.h"
"incl/pricingProblem/segmentBased/connectionNetwork/segmentConnectionNetworkNodes.h"
"incl/pricingProblem/segmentBased/centralisedNetwork/segmentCentralisedNetwork.h"
"incl/pricingProblem/segmentBased/centralisedNetwork/segmentCentralisedNetworkArcs.h"
"incl/pricingProblem/segmentBased/centralisedNetwork/segmentCentralisedNetworkNodes.h"
)

# Add the library, including all source files:
add_library(${libname} STATIC ${sourceFiles} ${headerFiles})

# Link the libraries
target_link_libraries(${libname} PUBLIC evaDataHandler highs::highs)
target_link_libraries(${libname} PRIVATE OpenMP::OpenMP_CXX)

# Everything linking to the source directory includes automatically the following header:
target_include_directories(${libname} 
PRIVATE 
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(${libname} 
PUBLIC 
	${HIGHS_INCLUDE_DIRS}
	"incl"
)

