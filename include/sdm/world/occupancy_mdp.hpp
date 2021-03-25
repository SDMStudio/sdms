/**
 * @file occupancy_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/decpomdp.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    //! \class  DecPOMDP
    template <typename oState, typename oAction>
    class OccupancyMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DecPOMDP> dpomdp_;
        oState istate_;
        oState cstate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        OccupancyMDP(std::shared_ptr<DecPOMDP> underlying_dpomdp);
        OccupancyMDP(std::shared_ptr<DecPOMDP> underlying_dpomdp, number hist_length);
        OccupancyMDP(std::string underlying_dpomdp);
        OccupancyMDP(std::string underlying_dpomdp, number hist_length);

        oState &getState();

        Reward getReward();
        double getDiscount() { return this->dpomdp_->getDiscount(); }
        void setDiscount(double discount) { return this->dpomdp_->setDiscount(discount); }

        DiscreteSpace<oAction> getActionSpace(oState);
        /**
         * @fn double getReward(Vector belief, number action);
         * @brief Get transformed reward from action and belief  
         */
        double getReward(oState belief, oAction action) const;

        oState &getInitialState();
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, oState ostate, oAction oaction, int t = 0) const;
        oState nextState(oState ostate, oAction oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>