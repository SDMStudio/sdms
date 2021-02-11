/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <random>
#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

//!
//! \file     stochastic_process.hpp
//! \author   David Albert
//! \brief    Generic Stochastic process class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This abstract class provide useful members for all stochastic processes.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  StochasticProcess
    class StochasticProcess
    {
    private:
        //! The internal state
        number internal_state_ = 0;

    protected:
        //! \brief The state space
        DiscreteSpace<number> state_space_;

        //! The initial state distribution
        Vector start_distrib_;

        //! \brief generator of starting state
        std::discrete_distribution<number> start_generator;

    public:
        StochasticProcess();
        // StochasticProcess(number);
        // StochasticProcess(const std::vector<double> &);
        // StochasticProcess(const Vector &);
        StochasticProcess(DiscreteSpace<number>);
        StochasticProcess(DiscreteSpace<number>, Vector );

        //! \fn       number getInternalState()
        //! \brief    Get the internal state.
        //! \return   the internal state
        number getInternalState() const;

        //! \fn       void setInternalState(number)
        //! \param    internal_state the new internal state
        //! \brief    Sets the internal state.
        void setInternalState(number);

        void setupStartGenerator();

        //! \fn       number init()
        //! \brief    Init processus and return initial sampled state
        //! \return   the initial state
        number init();

        const Vector &getStartDistrib() const;

        void setStartDistrib(const std::vector<double> &);

        void setStartDistrib(const Vector &);

        DiscreteSpace<number> getStateSpace() const;

        //! \fn       state getNumStates() const
        //! \brief    Returns the number of states
        //! \return   state number
        number getNumStates() const;
    };
} // namespace sdm
