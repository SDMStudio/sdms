/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/public/world.hpp>

//!
//! \file     gym_interface.hpp
//! \author   David Albert
//! \brief    GymInterface class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This abstract class provide an interface based on gym environment.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  GymInterface
    //! \brief Provides a Gym like interface
    template <typename TDecProcess = POSG>
    class InteractiveWorld : public GymInterface<typename TDecProcess::observation_space_type, typename TDecProcess::action_space_type>
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

        //! \param intern_formalism problem to interact with
        InteractiveWorld(std::shared_ptr<TDecProcess>);

        InteractiveWorld(const TDecProcess &);

        InteractiveWorld(const std::string &);

        observation_type reset();

        std::tuple<observation_type, std::vector<double>, bool> step(action_type ja); // std::tuple<std::vector<number>, std::vector<double>, bool, map>
    };
} // namespace sdm
#include <sdm/world/interactive_world.tpp>