#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/base/pomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief Public interface for every belief mdp.
     */
    class BeliefMDPInterface : virtual public SolvableByHSVI
    {
    public:
        /** @brief Get the address of the underlying POMDP */
        virtual std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const = 0;

        /** @brief Get the address of the underlying BeliefMDP */
        virtual std::shared_ptr<BeliefMDPInterface> getUnderlyingBeliefMDP() = 0;
    };
} // namespace sdm