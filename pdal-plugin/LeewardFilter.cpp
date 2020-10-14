#include "LeewardFilter.hpp"
#include "leeward.h"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.leeward",
        "Lidar Equation Engine With Already Racked Derivatives",
        ""};

    CREATE_SHARED_STAGE(LeewardFilter, s_info)

    std::string LeewardFilter::getName() const { return s_info.name; }

    PointViewSet LeewardFilter::run(PointViewPtr input)
    {
        PointViewSet output;
        output.insert(input);
        auto leeward = leeward_new("foobar", 42);
        if (!leeward)
        {
            throw pdal_error("Error when creating leeward, exiting...");
        }
        leeward_delete(leeward);
        return output;
    }
} // namespace pdal