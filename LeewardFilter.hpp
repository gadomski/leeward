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

        std::string m_sbet;
        std::string m_config;
        Dimension::Id m_horizontalUncertainty;
        Dimension::Id m_verticalUncertainty;
        Dimension::Id m_totalUncertainty;
        Dimension::Id m_incidenceAngle;
    };
} // namespace pdal