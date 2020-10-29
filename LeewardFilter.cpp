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
        this->m_sigmaX = layout->registerOrAssignDim("SigmaX", Dimension::Type::Float);
        this->m_sigmaY = layout->registerOrAssignDim("SigmaY", Dimension::Type::Float);
        this->m_sigmaHorizontal = layout->registerOrAssignDim("SigmaHorizontal", Dimension::Type::Float);
        this->m_sigmaVertical = layout->registerOrAssignDim("SigmaVertical", Dimension::Type::Float);
        this->m_sigmaMagnitude = layout->registerOrAssignDim("SigmaMagnitude", Dimension::Type::Float);
        this->m_incidenceAngle = layout->registerOrAssignDim("IncidenceAngle", Dimension::Type::Float);
    }

    void LeewardFilter::addArgs(ProgramArgs &args)
    {
        args.add("sbet_path", "Path to the sbet file", this->m_sbet_path);
        args.add("config_path", "Path to the sbet file", this->m_config_path);
    }

    void LeewardFilter::filter(PointView &view)
    {
        if (this->m_sbet_path.empty())
        {
            throw pdal_error("No SBET path provided, exiting...");
        }
        if (this->m_config_path.empty())
        {
            throw pdal_error("No config path provided, exiting...");
        }
        auto leeward = leeward_new(this->m_sbet_path.c_str(), this->m_config_path.c_str());
        if (!leeward)
        {
            throw pdal_error("Error when creating leeward, exiting...");
        }
        for (PointId id = 0; id < view.size(); ++id)
        {
            auto x = view.getFieldAs<double>(Dimension::Id::X, id);
            auto y = view.getFieldAs<double>(Dimension::Id::Y, id);
            auto z = view.getFieldAs<double>(Dimension::Id::Z, id);
            auto scan_angle = view.getFieldAs<float>(Dimension::Id::ScanAngleRank, id);
            auto gps_time = view.getFieldAs<float>(Dimension::Id::GpsTime, id);
            auto nx = view.getFieldAs<float>(Dimension::Id::NormalX, id);
            auto ny = view.getFieldAs<float>(Dimension::Id::NormalY, id);
            auto nz = view.getFieldAs<float>(Dimension::Id::NormalZ, id);
            auto tpu = leeward_tpu(leeward, x, y, z, scan_angle, gps_time, nx, ny, nz);
            view.setField(this->m_sigmaX, id, tpu->sigma_x);
            view.setField(this->m_sigmaY, id, tpu->sigma_y);
            view.setField(this->m_sigmaHorizontal, id, tpu->sigma_horizontal);
            view.setField(this->m_sigmaVertical, id, tpu->sigma_vertical);
            view.setField(this->m_sigmaMagnitude, id, tpu->sigma_magnitude);
            view.setField(this->m_incidenceAngle, id, tpu->incidence_angle * 180 / M_PI);
            leeward_tpu_delete(tpu);
        }
        leeward_delete(leeward);
    }
} // namespace pdal