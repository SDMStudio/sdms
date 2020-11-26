/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

#include <sdm/world/po_process.hpp>
#include <sdm/world/decision_process.hpp>

#include <sdm/core/discrete_space.hpp>
#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>

//!
//! \file     posg.hpp
//! \author   David Albert
//! \brief    Partially observable stochastic game class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate a POSG.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  DecisionProcess
    //! \brief Decision process
    class POSG : public SG, public POProcess
    {
    protected:
        ObservationDynamics o_dynamics_;

    public:
        POSG();

        //! \param stochastic_game stochastic game
        //! \brief Construct a POSG from a SG (i.e. build observation function as identity)
        POSG(const SG &stochastic_game);

        //! \param filename name of the file describing the POSG
        POSG(const std::string &filename);

        //! \param    num_states number of states
        //! \param    num_agents number of agents
        POSG(number, number);

        //! \param    num_states number of states
        //! \param    num_agents number of agents
        //! \param    num_actions number of actions for each agent
        //! \param    num_observations number of observations for each agent
        POSG(number, number, const std::vector<number> &, const std::vector<number> &);

        //! \param    state_sp the state space
        //! \param    agent_sp the agent space
        POSG(const DiscreteSpace &, const DiscreteSpace &);

        //! \param    state_sp the state space
        //! \param    agent_sp the agent space
        //! \param    action_sp the multi action space
        //! \param    obs_sp the multi observation space
        //! \param    s_dyn state dynamics
        //! \param    o_dyn observation dynamics
        //! \param    rews reward functions for each agent
        //! \param    start_distrib start distribution (optional)
        POSG(const DiscreteSpace &, const DiscreteSpace &, const MultiDiscreteSpace &, const MultiDiscreteSpace &,
             const StateDynamics &, const ObservationDynamics &, const std::vector<Reward> &,  const Vector &);

        const ObservationDynamics& getObsDynamics() const;

        //! \fn       double getObservationProbability(number, number, number) const
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \param    state a specific state
        //! \brief    Returns probability
        //! \return   value
        double getObservationProbability(number jaction, number jobservation, number state) const;

        double getObservationProbability(std::vector<number> jaction, std::vector<number> jobservation, number state) const;

        //! \fn       const Matrix& getObservations(action)
        //! \param    jaction a specific joint action (as single one)
        //! \brief    Returns matrix of probability observations for the pre-defined action.
        const Matrix &getObservations(number jaction) const;

        //! \fn       const Matrix& getObservations(action)
        //! \param    jaction a specific joint action
        //! \brief    Returns matrix of probability observations for the pre-defined action.
        const Matrix &getObservations(std::vector<number> jaction) const;

        //! \fn       value getDynamics(state, action, observation, state) const
        //! \param    cstate a specific state (timestep t)
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \param    nstate a specific state (timestep t+1)
        //! \brief    Returns probability
        //! \return   value
        double getDynamics(number cstate, number jaction, number jobservation, number nstate) const;

        //! \fn       const matrix& getDynamics(action, observation) const
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \brief    Returns transition matrix
        //! \return   const matrix&
        const Matrix &getDynamics(number jaction, number jobservation) const;
    };

    typedef POSG POStochasticGame;
    typedef POSG PartObsStochasticGame;
    typedef POSG PartiallyObservableStochasticGame;
} // namespace sdm
