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

    void LeewardFilter::addDimensions(PointLayoutPtr layout)
    {
        this->m_horizontalUncertainty = layout->registerOrAssignDim("HorizontalUncertainty", Dimension::Type::Float);
        this->m_verticalUncertainty = layout->registerOrAssignDim("VerticalUncertainty", Dimension::Type::Float);
        this->m_totalUncertainty = layout->registerOrAssignDim("TotalUncertainty", Dimension::Type::Float);
        this->m_incidenceAngle = layout->registerOrAssignDim("IncidenceAngle", Dimension::Type::Float);
    }

    void LeewardFilter::addArgs(ProgramArgs &args)
    {
        args.add("sbet", "Path to the sbet file", this->m_sbet);
        args.add("config", "Path to the sbet file", this->m_config);
    }

    void LeewardFilter::filter(PointView &view)
    {
        if (this->m_sbet.empty())
        {
            throw pdal_error("filters.leeward: no sbet path provided, exiting...");
        }
        if (this->m_config.empty())
        {
            throw pdal_error("filters.leeward: no config path provided");
        }
        auto leeward = leeward_new(this->m_sbet.c_str(), this->m_config.c_str());
        if (!leeward)
        {
            throw pdal_error("filters.leeward: error when creating leeward object");
        }
        for (PointId id = 0; id < view.size(); ++id)
        {
            struct LeewardPoint point;
            point.x = view.getFieldAs<double>(Dimension::Id::X, id);
            point.y = view.getFieldAs<double>(Dimension::Id::Y, id);
            point.z = view.getFieldAs<double>(Dimension::Id::Z, id);
            point.scan_angle = view.getFieldAs<float>(Dimension::Id::ScanAngleRank, id);
            point.time = view.getFieldAs<float>(Dimension::Id::GpsTime, id);
            struct LeewardNormal normal;
            // TODO handle missing normal fields
            normal.x = view.getFieldAs<float>(Dimension::Id::NormalX, id);
            normal.y = view.getFieldAs<float>(Dimension::Id::NormalY, id);
            normal.z = view.getFieldAs<float>(Dimension::Id::NormalZ, id);
            auto measurement = leeward_measurement(leeward, point, normal);
            if (measurement)
            {
                view.setField(this->m_horizontalUncertainty, id, measurement->horizontal_uncertainty);
                view.setField(this->m_verticalUncertainty, id, measurement->vertical_uncertainty);
                view.setField(this->m_totalUncertainty, id, measurement->total_uncertainty);
                view.setField(this->m_incidenceAngle, id, measurement->incidence_angle * 180 / M_PI);
                leeward_measurement_free(measurement);
            }
            else
            {
                std::cerr << "filters.leeward: error when creating measurement at time " << point.time << ", skipping..." << std::endl;
            }
        }
        leeward_free(leeward);
    } // namespace pdal
} // namespace pdal