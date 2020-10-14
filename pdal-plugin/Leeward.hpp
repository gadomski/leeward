#pragma once

#include <pdal/Filter.hpp>
#include <pdal/pdal_internal.hpp>

namespace pdal
{
    class PDAL_DLL Leeward : public Filter
    {
    public:
        Leeward() : Filter() {}

        std::string getName() const;

    private:
    };
} // namespace pdal