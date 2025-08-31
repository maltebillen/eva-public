#include <iostream>

#include "evaExceptions.h"
#include "evaDataHandler.h"
#include "evaOptimiser.h"

int main(int argc, char* argv[])
{
	try
	{
		std::cout << "Starting EVA Optimiser." << std::endl;

		// Read in the values passed in the command line.
		if (argc < 2)
		{
			throw eva::DataError("main", "EVA.cpp: console input missing.");
		}
		eva::Types::CommandInput commandInputValues;
		commandInputValues.PathToData = argv[1];
		commandInputValues.PathToConfig = argv[2];
	
		// Read in the data, given the command line input:
		eva::DataHandler dataHandler(commandInputValues);

		// Run the algorithm:
		eva::Optimiser optimiser(dataHandler);
		optimiser.run(); // In the future, read the algorithm type from the input console.

		// Store the final stats:
		dataHandler.storeStatsVehicles();
		dataHandler.storeStatsChargers();
		dataHandler.storeStatsSchedules();

		// Finally, store the schedule in a .csv:
		dataHandler.writeOutputToCsv();
	}
	catch (eva::DataError& dataError)
	{
		std::cerr << "Message: " + std::string(dataError.what());
	}
	catch (eva::LogicError& logicError)
	{
		std::cerr << "Message: " + std::string(logicError.what());
	}
	return 0;
}