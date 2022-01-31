#pragma once

#include <sdm/core/action/action.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
    class NDPOMDPNaming
    {
    public:
        std::string getTransitionName(const std::shared_ptr<State> &x, const std::shared_ptr<State> &y);

    };
}