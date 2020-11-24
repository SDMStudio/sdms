#pragma once

#include <random>

#include <sdm/types.hpp>

namespace sdm
{
    namespace common
    {
        void logo();
        std::default_random_engine &global_urng();
    } // namespace common
} // namespace sdm
