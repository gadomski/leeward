#include "BodyFrameFilter.hpp"
#include "leeward.h"

namespace pdal
{
    static PluginInfo const s_info{
        "filters.body_frame",
        "Add body frame coordinates",
        ""};

    CREATE_SHARED_STAGE(BodyFrameFilter, s_info)

    std::string BodyFrameFilter::getName() const { return s_info.name; }

    void BodyFrameFilter::addDimensions(PointLayoutPtr layout)
    {
        this->m_bodyFrameX = layout->registerOrAssignDim("BodyFrameX", Dimension::Type::Float);
        this->m_bodyFrameY = layout->registerOrAssignDim("BodyFrameY", Dimension::Type::Float);
        this->m_bodyFrameZ = layout->registerOrAssignDim("BodyFrameZ", Dimension::Type::Float);
        this->m_roll = layout->registerOrAssignDim("Roll", Dimension::Type::Float);
        this->m_pitch = layout->registerOrAssignDim("Pitch", Dimension::Type::Float);
        this->m_yaw = layout->registerOrAssignDim("Yaw", Dimension::Type::Float);
    }

    void BodyFrameFilter::addArgs(ProgramArgs &args)
    {
        args.add("sbet", "Path to the sbet file", this->m_sbet);
        args.add("config", "Path to the sbet file", this->m_config);
        args.add("offset", "Time offset", this->m_offset, float(0.0));
    }

    void BodyFrameFilter::filter(PointView &view)
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
            lidar.time = view.getFieldAs<float>(Dimension::Id::GpsTime, id) + m_offset;
            auto body_frame = leeward_body_frame(leeward, &lidar);
            if (body_frame)
            {
                view.setField(this->m_bodyFrameX, id, body_frame->x);
                view.setField(this->m_bodyFrameY, id, body_frame->y);
                view.setField(this->m_bodyFrameZ, id, body_frame->z);
                view.setField(this->m_roll, id, body_frame->roll);
                view.setField(this->m_pitch, id, body_frame->pitch);
                view.setField(this->m_yaw, id, body_frame->yaw);
                leeward_body_frame_free(body_frame);
            }
            else
            {
                std::cerr << "Error when creating uncertainty, skipping point..." << std::endl;
            }
        }
        leeward_free(leeward);
    } // namespace pdal
} // namespace pdal