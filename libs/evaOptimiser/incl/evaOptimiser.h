#ifndef EVA_OPTIMISER_H
#define EVA_OPTIMISER_H

#include "evaDataHandler.h"

namespace eva
{
	class Optimiser
	{
		// ATTRIBUTES 

		DataHandler& _dataHandler;

		std::vector<std::vector<BoostScheduleNode>> _vecSchedule;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTOR

		Optimiser() = delete;
		Optimiser(DataHandler& dataHandler) :
			_dataHandler(dataHandler)
		{}

		void run();
	};
};


#endif // !EVA_OPTIMISER_H