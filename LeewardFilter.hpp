#pragma once

#include <pdal/Filter.hpp>
#include <pdal/pdal_internal.hpp>

namespace pdal
{
    class PDAL_DLL LeewardFilter : public Filter
    {
    public:
        LeewardFilter() : Filter() {}

        std::string getName() const;

    private:
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void addArgs(ProgramArgs &args);
        virtual void filter(PointView &view);

        std::string m_sbet_path;
        std::string m_config_path;
        Dimension::Id m_sigmaX;
        Dimension::Id m_sigmaY;
        Dimension::Id m_sigmaHorizontal;
        Dimension::Id m_sigmaVertical;
        Dimension::Id m_sigmaMagnitude;
        Dimension::Id m_incidenceAngle;
    };
} // namespace pdal