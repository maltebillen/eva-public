#include "evaConstants.h"
#include "evaExceptions.h"

#include <cmath>
#include <iomanip>
#include <ctime>
#include <time.h>
#include <sstream>
#include <limits>
#include <algorithm>

using namespace eva;

const Types::Index Constants::BIG_INDEX = std::numeric_limits<Types::Index>::max();
const int32_t Constants::BIG_INTEGER = std::numeric_limits<int32_t>::max();
const uint32_t Constants::BIG_UINTEGER = std::numeric_limits<uint32_t>::max();
const Types::BatteryCharge Constants::BIG_BATTERYCHARGE = std::numeric_limits<Types::BatteryCharge>::max();
const double Constants::BIG_DOUBLE = std::numeric_limits<double>::max();
const Types::DateTime Constants::MAX_TIMESTAMP = std::numeric_limits<Types::DateTime>::max();
const double Constants::EPS = 0.00001;

const bool Helper::compare_floats_equal(const double& a, const double& b)
{
	if (std::fabs(a - b) <= Constants::EPS) {
		return true;
	}
	return false;
}

const bool Helper::compare_floats_smaller(const double& a, const double& b)
{
	if ((a - b) < -Constants::EPS) {
		return true;
	}
	return false;
}

const bool Helper::compare_floats_smaller_equal(const double& a, const double& b)
{
	return !compare_floats_smaller(b, a);
}

const bool eva::Helper::compare_is_subset(const std::vector<Types::AccessType> &a, const std::vector<Types::AccessType> &b)
{
	// Checks if b is a subset of a.
	// e.g. [1, 1, 1, 0] has subset [0, 1, 1, 0], 
	// but. [1, 1, 0, 0] has NOT subset [0, 1, 1, 0], 
	if (a.size() != b.size()) return false;

	for(Types::Index idx = 0; idx < a.size(); ++idx)
	{
		// must be a[idx] >= b[idx]
		if(a[idx] < b[idx])
		{
			return false;
		}
	}

    return true;
}

const Types::DateTime Helper::StringToDateTime(const std::string &str)
{
	struct std::tm tm;

	// Read the Types::DateTime from String in format: %Y-%m-%d %H:%M:%S
	// Always reads the date in GMT format, and ignores any timezone offsets/
	if (strptime(str.c_str(), "%Y-%m-%d %H:%M:%S", &tm) == NULL)
	{
		throw InvalidArgumentError("Helper::StringToDateTime", "Date: " + str + " (string) not in correct date format or it is not specified.");
	}

	return Types::DateTime(timegm(&tm));
}

const std::string Helper::DateTimeToString(const Types::DateTime& dt)
{
	char res[sizeof "YYYY-MM-DD HH:MM:SS+ZZZZ"];
	strftime(res, sizeof res, "%F %T%z", std::gmtime(&dt));
	std::string str_res = res;

	return str_res;
}

const std::string eva::Helper::DurationToString(const uint32_t& durInSeconds)
{
	uint32_t hh, mm, ss, mod;
	std::string strhh, strmm, strss;
	// 1. HH:
	hh = std::floor(durInSeconds / (60 * 60));
	strhh = hh < 10 ? "0" + std::to_string(hh) : std::to_string(hh);
	mod = durInSeconds % (60 * 60);

	// 2. MM: 
	mm = std::floor(mod / 60);
	strmm = mm < 10 ? "0" + std::to_string(mm) : std::to_string(mm);
	mod = mod % 60;

	// 3. SS:
	ss = mod;
	strss = ss < 10 ? "0" + std::to_string(ss) : std::to_string(ss);

	return "T" + strhh + ":" + strmm + ":" + strss;
}

const int64_t Helper::diffDateTime(const Types::DateTime& beginn, const Types::DateTime& end)
{
	// Return the time difference in seconds.
	return std::difftime(end, beginn);
}

const int64_t Helper::diffDateTime(const Types::DateTimeInterval& interval)
{
	// Return the time difference in seconds.
	return std::difftime(interval.UpperBound, interval.LowerBound);
}

const Types::DateTime Helper::roundToNearestMinute(const Types::DateTime& dt)
{
	uint32_t res = dt % 60;// round to nearest minute ( = 60 seconds).

	if (res < 30)
	{
		return dt - res;
	}
	else
	{
		return dt - res + 60;
	}
}

const uint32_t eva::Helper::intDivisionRoundUp(const uint32_t& x, const uint32_t& y)
{
	return (x + y - 1) / y;
}

const uint32_t eva::Helper::intDivisionRoundDown(const uint32_t& x, const uint32_t& y)
{
	return x / y;
}

const bool eva::Helper::stringToBoolean(const std::string& str)
{
	auto bool_str = str;
	std::transform(bool_str.begin(), bool_str.end(), bool_str.begin(), ::tolower);

	if (bool_str == "true" || bool_str == "1") {
		return true;
	}

	return false;
}
