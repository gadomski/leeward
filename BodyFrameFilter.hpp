#pragma once

#include <pdal/Filter.hpp>
#include <pdal/pdal_internal.hpp>

namespace pdal
{
    class PDAL_DLL BodyFrameFilter : public Filter
    {
    public:
        BodyFrameFilter() : Filter() {}

        std::string getName() const;

    private:
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void addArgs(ProgramArgs &args);
        virtual void filter(PointView &view);

        std::string m_sbet;
        std::string m_config;
        float m_offset;
        Dimension::Id m_bodyFrameX;
        Dimension::Id m_bodyFrameY;
        Dimension::Id m_bodyFrameZ;
        Dimension::Id m_roll;
        Dimension::Id m_pitch;
        Dimension::Id m_yaw;
    };
} // namespace pdal