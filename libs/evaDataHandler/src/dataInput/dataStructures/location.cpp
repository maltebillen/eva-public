#include "incl/dataInput/dataStructures/location.h"

const eva::Location::LocationType eva::Location::string2type(const std::string& str)
{
    if (str == "Stop")
        return LocationType::Stop;

    if (str == "Charger")
        return LocationType::Charger;
        
    if (str == "Maintenance")
        return LocationType::Maintenance;

    return LocationType::Undefined;
}

