/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/public/world.hpp>
#include <sdm/core/space.hpp>
#include <sdm/core/multi_discrete_space.hpp>

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
    class GymInterface : public World
    {
    protected:
        std::shared_ptr<Space> observation_space_;
        std::shared_ptr<Space> action_space_;
    public:
        GymInterface(std::shared_ptr<Space> , std::shared_ptr<Space> );
        // GymInterface(MultiDiscreteSpace, MultiDiscreteSpace);

        std::shared_ptr<Space> getObsSpace() const;
        std::shared_ptr<Space> getActionSpace() const;

        virtual std::vector<number> reset() = 0;
        virtual std::tuple<std::vector<number>, std::vector<double>, bool> step(std::vector<action> a) = 0; // std::tuple<Observation, Reward, bool, map>
    };
} // namespace sdm
