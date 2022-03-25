/**
 * @file discrete_mdp.hpp
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
#include <sdm/core/space/space.hpp>
#include <sdm/world/base/sg_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes.
     *
     */
    class POSGInterface : virtual public MPOMDPInterface,
                          virtual public SGInterface
    {
    };

} // namespace sdm