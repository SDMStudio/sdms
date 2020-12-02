/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/space.hpp>
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
    class InteractiveWorld : public GymInterface
    {
    protected:
        //! \brief The current timestep
        number ctimestep_ = 0;

    public:
        std::shared_ptr<POSG> internal_formalism_;

        //! \param intern_formalism problem to interact with
        InteractiveWorld(std::shared_ptr<POSG>);

        InteractiveWorld(const POSG &);

        InteractiveWorld(const std::string &);

        std::vector<number> reset();

        std::tuple<std::vector<number>, std::vector<double>, bool> step(std::vector<number> ja); // std::tuple<std::vector<number>, std::vector<double>, bool, map>
    };
} // namespace sdm
