#include "Leeward.hpp"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.leeward",
        "Lidar Equation Engine With Already Racked Derivatives",
        ""};

    CREATE_SHARED_STAGE(Leeward, s_info)

    std::string Leeward::getName() const { return s_info.name; }
} // namespace pdal