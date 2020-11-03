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
        this->m_xUncertainty = layout->registerOrAssignDim("XUncertainty", Dimension::Type::Float);
        this->m_yUncertainty = layout->registerOrAssignDim("YUncertainty", Dimension::Type::Float);
        this->m_horizontalUncertainty = layout->registerOrAssignDim("HorizontalUncertainty", Dimension::Type::Float);
        this->m_verticalUncertainty = layout->registerOrAssignDim("VerticalUncertainty", Dimension::Type::Float);
        this->m_uncertainty = layout->registerOrAssignDim("Uncertainty", Dimension::Type::Float);
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
            throw pdal_error("No SBET path provided, exiting...");
        }
        if (this->m_config.empty())
        {
            throw pdal_error("No config path provided, exiting...");
        }
        auto leeward = leeward_new(this->m_sbet.c_str(), this->m_config.c_str());
        if (!leeward)
        {
            throw pdal_error("Error when creating leeward, exiting...");
        }
        for (PointId id = 0; id < view.size(); ++id)
        {
            struct LeewardLidar lidar;
            lidar.x = view.getFieldAs<double>(Dimension::Id::X, id);
            lidar.y = view.getFieldAs<double>(Dimension::Id::Y, id);
            lidar.z = view.getFieldAs<double>(Dimension::Id::Z, id);
            lidar.scan_angle = view.getFieldAs<float>(Dimension::Id::ScanAngleRank, id);
            lidar.time = view.getFieldAs<float>(Dimension::Id::GpsTime, id);
            struct LeewardNormal normal;
            normal.x = view.getFieldAs<float>(Dimension::Id::NormalX, id);
            normal.y = view.getFieldAs<float>(Dimension::Id::NormalY, id);
            normal.z = view.getFieldAs<float>(Dimension::Id::NormalZ, id);
            auto uncertainty = leeward_uncertainty_with_normal(leeward, &lidar, &normal);
            if (!uncertainty)
            {
                throw pdal_error("Error when creating uncertainty, exiting...");
            }
            view.setField(this->m_xUncertainty, id, uncertainty->x);
            view.setField(this->m_yUncertainty, id, uncertainty->y);
            view.setField(this->m_horizontalUncertainty, id, uncertainty->horizontal);
            view.setField(this->m_verticalUncertainty, id, uncertainty->vertical);
            view.setField(this->m_uncertainty, id, uncertainty->total);
            view.setField(this->m_incidenceAngle, id, uncertainty->incidence_angle * 180 / M_PI);
            leeward_uncertainty_free(uncertainty);
        }
        leeward_free(leeward);
    }
} // namespace pdal