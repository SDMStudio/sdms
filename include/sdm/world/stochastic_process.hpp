/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/discrete_space.hpp>
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
        DiscreteSpace state_space_;

        //! The initial state distribution
        std::shared_ptr<Vector> start_distrib_ = std::shared_ptr<Vector>(nullptr);

        //! \fn       number getState()
        //! \brief    Get the internal state.
        //! \return   the internal state
        number getState() const;

        //! \fn       void setState(number)
        //! \param    internal_state the new internal state
        //! \brief    Sets the internal state.
        void setState(number);

    public:
        StochasticProcess();
        StochasticProcess(number);
        StochasticProcess(const std::vector<double> &);
        StochasticProcess(const Vector &);
        StochasticProcess(const DiscreteSpace &);
        StochasticProcess(const DiscreteSpace &, const Vector &);

        const Vector &getStartDistrib() const;
        void setStartDistrib(const std::vector<double> &);
        void setStartDistrib(const Vector &);

        const DiscreteSpace &getStateSpace() const;

        //! \fn       state getNumStates() const
        //! \brief    Returns the number of states
        //! \return   state number
        number getNumStates() const;

        //! \fn       state getStateIndex(const std::string&)
        //! \param    const std::string& a state name
        //! \brief    Returns the index of the state
        number getStateIndex(const std::string &) const;

        //! \fn       std::string getStateName(state)
        //! \param    state  index
        //! \brief    Returns the name associated with the state index
        std::string getStateName(number) const;

        //! \fn       void setStatesNames(std::vector<std::string>&)
        //! \param    const std::vector<std::string>&
        //! \brief    Sets the names of states.
        void setStatesNames(const std::vector<std::string> &);
    };
} // namespace sdm
