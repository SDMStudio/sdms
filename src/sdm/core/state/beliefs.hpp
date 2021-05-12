/**
 * @file beliefs.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains the generic belief classes.
 * @version 0.1
 * @date 14/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_vector.hpp>
#include <sdm/core/state/belief_state_graph.hpp>

namespace sdm
{
    template <typename action_t, typename obs_t>
    using BeliefStateGraph_p = std::shared_ptr<BeliefStateGraph<BeliefStateVector, action_t, obs_t>>;
} // namespace sdm