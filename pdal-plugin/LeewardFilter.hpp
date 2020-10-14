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
        virtual PointViewSet run(PointViewPtr view);
    };
} // namespace pdal