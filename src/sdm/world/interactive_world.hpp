/**
 * @file interactive_world.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 04/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/public/world.hpp>

namespace sdm
{
    //! \class  GymInterface
    //! \brief Provides a Gym like interface
    template <typename TDecProcess = POSG>
    class InteractiveWorld : public GymInterface<typename TDecProcess::observation_space_type, typename TDecProcess::action_space_type, typename TDecProcess::reward_function_type>
    {
    protected:
        //! \brief The current timestep
        number ctimestep_ = 0;
        std::shared_ptr<TDecProcess> internal_formalism_;

    public:
        using observation_space_type = typename TDecProcess::observation_space_type;
        using observation_type = typename observation_space_type::value_type;

        using action_space_type = typename TDecProcess::action_space_type;
        using action_type = typename action_space_type::value_type;

        using reward_function_type = typename TDecProcess::reward_type;
        using reward_type = typename reward_type::value_type;

        //! \param intern_formalism problem to interact with
        InteractiveWorld(TDecProcess *);

        InteractiveWorld(const TDecProcess &);

        InteractiveWorld(const std::string &);

        observation_type reset();

        std::tuple<observation_type, reward_type, bool> step(action_type ); 
    };
} // namespace sdm
#include <sdm/world/interactive_world.tpp>