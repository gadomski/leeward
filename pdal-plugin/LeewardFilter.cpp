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
        std::cout << "hi from C" << std::endl;
        leeward_capi_test();
        return output;
    }
} // namespace pdal