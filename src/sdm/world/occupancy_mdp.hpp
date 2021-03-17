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
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    class DiscreteDecPOMDP;

    /**
     * @brief 
     * 
     * @tparam oState 
     * @tparam oAction 
     */
    template <typename oState, typename oAction>
    class OccupancyMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;
        oState istate_;
        oState cstate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp);
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length);
        OccupancyMDP(std::string underlying_dpomdp);
        OccupancyMDP(std::string underlying_dpomdp, number hist_length);

        oState &getState();

        std::shared_ptr<Reward> getReward() const;
        double getDiscount() { return this->dpomdp_->getDiscount(); }
        void setDiscount(double discount) { return this->dpomdp_->setDiscount(discount); }
        
        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);
        
        /**
         * @brief Get transformed reward from action and belief  
         */
        double getReward(const oState &ostate, const oAction &oaction) const;

        oState getInitialState();
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>