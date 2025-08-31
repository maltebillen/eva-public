#ifndef EVA_GLOBAL_CONSTANTS_H
#define EVA_GLOBAL_CONSTANTS_H

#include <stdint.h>
#include <string>
#include <ctime>
#include <vector>

namespace eva {
	struct Types
	{
		enum class AlgorithmType : uint8_t {
			PRICE_AND_BRANCH = 0,
			BRANCH_AND_PRICE_BEST = 1,
			BRANCH_AND_PRICE_DEPTH = 2,
			DIVING_HEURISTIC = 3,
			DIVING_THEN_BFBNP = 4,
			DIVING_THEN_DFBNP = 5
		};

		enum class PricingProblemType : uint8_t {
			TIME_SPACE_NETWORK = 0,
			CONNECTION_SEGMENT_NETWORK = 1,
			CENTRALISED_SEGMENT_NETWORK = 2
		};

		enum class AccessType : uint8_t
		{
			NOT_ALLOWED = 0,
			ALLOWED = 1
		};

		typedef uint32_t Index;
		typedef int32_t BatteryCharge;
		typedef std::time_t DateTime;

		struct DateTimeInterval
		{
			DateTime LowerBound;
			DateTime UpperBound;

			DateTimeInterval(const DateTime& dt) :
				LowerBound(dt),
				UpperBound(dt)
			{}

			DateTimeInterval(const DateTime& lb, const DateTime& ub) :
				LowerBound(lb),
				UpperBound(ub)
			{}
		};

		struct CommandInput
		{
			std::string PathToData;
			std::string PathToConfig;
			AlgorithmType CodeAlgorithmType;
		};
	};

	struct Constants
	{
		static const Types::Index BIG_INDEX;
		static const int32_t BIG_INTEGER;
		static const uint32_t BIG_UINTEGER;
		static const Types::BatteryCharge BIG_BATTERYCHARGE;
		static const double BIG_DOUBLE;
		static const Types::DateTime MAX_TIMESTAMP;
		static const double EPS;
	};

	struct Helper
	{
		// double(a) == double(b)
		const static bool compare_floats_equal(const double& a, const double& b);

		// double(a) < double(b)
		const static bool compare_floats_smaller(const double& a, const double& b);

		// double(a) <= double(b)
		const static bool compare_floats_smaller_equal(const double& a, const double& b);

		// set(a) >= set(b)
		const static bool compare_is_subset(const std::vector<Types::AccessType>& a, const std::vector<Types::AccessType>& b);

		const static Types::DateTime StringToDateTime(const std::string& str);
		const static std::string DateTimeToString(const Types::DateTime& dt);
		const static std::string DurationToString(const uint32_t& durInSeconds);
		const static int64_t diffDateTime(const Types::DateTime& beginn, const Types::DateTime& end);
		const static int64_t diffDateTime(const Types::DateTimeInterval& interval);
		const static Types::DateTime roundToNearestMinute(const Types::DateTime& dt);
		const static uint32_t intDivisionRoundUp(const uint32_t& x, const uint32_t& y);
		const static uint32_t intDivisionRoundDown(const uint32_t& x, const uint32_t& y);
		const static bool stringToBoolean(const std::string& str);

		template <class T> static std::vector<T> ConcatVectors(const std::vector<T>& a, const std::vector<T>& b);
	};
}

// This includes the implementation of all template C++ classes:
#include "evaConstants.tcc"

#endif /* EVA_GLOBAL_CONSTANTS_H */