#pragma once

#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>


namespace sdm
{
    class Belief2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> pomdp_vf_;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction> pomdp_vf);

        double operatorMPOMDP(const std::shared_ptr<State> &, const number &);        
        double operator()(const std::shared_ptr<State> &, const number &);
        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &, const number &);

        bool isPomdpAvailable();
        bool isMdpAvailable();

    };
} // namespace sdm
