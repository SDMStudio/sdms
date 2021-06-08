/**
 * @file mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/reward_interface.hpp>
#include <sdm/core/dynamics/state_dynamics_interface.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MMDP : virtual public MDP, virtual public MMDPInterface
    {
    public:
        MMDP(double horizon,
             double discount,
             const std::shared_ptr<Space> &state_space,
             const std::shared_ptr<Space> &action_space,
             const std::shared_ptr<RewardInterface> &reward,
             const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
             const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib);

        std::shared_ptr<Space> getActionSpace(number t = 0) const;
        std::shared_ptr<Space> getActionSpace(number agent_id, number t) const;
    };

} // namespace sdm