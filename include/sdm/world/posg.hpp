/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>

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

        //! \brief Map (state, jaction) to probability of (next_state, next_observation) --> i.e. s_{t+1}, o_{t+1} ~ P(S_{t+1}, O_{t+1}  | S_t = s, A_t = a )
        std::unordered_map<number, std::unordered_map<number, std::discrete_distribution<std::size_t>>> dynamics_generator;

        //! \brief map integer representing joint state/observation to this couple (state, observation)
        std::unordered_map<number, std::pair<number, number>> encoding;

        //! \fn void setupDynamicsGenerator()
        //! \brief Setup de dynamics generator
        void setupDynamicsGenerator();


    public:
        POSG();

        POSG(const POSG &posg);

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
             const StateDynamics &, const ObservationDynamics &, const std::vector<Reward> &, const Vector &);


        //! \fn std::tuple<double, observation, state> getDynamicsGenerator(state x, action a)
        //! \param state the current state
        //! \param jaction the joint action
        //! \return a tuple containing reward, next_osbservation and next_state
        std::tuple<std::vector<double>, observation, state> getDynamicsGenerator(number x, number a) const;

        const ObservationDynamics &getObsDynamics() const;

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

        //! \fn std::string toStdFormat()
        //! \brief Encodes POSG class into a string (standard .posg or .dpomdp or .zsposg format).
        //! \return the resulting standard file format like string
        std::string toStdFormat() const;

        //! \fn std::string toXML()
        //! \brief Encodes POSG class into a string (XML format).
        //! \return the resulting XML like string
        std::string toXML() const;

        //! \fn std::string toStdFormat()
        //! \brief Encodes POSG class into a string (JSON format).
        //! \return the resulting JSON like string
        std::string toJSON() const;

        //! \fn void generateFile(std::string)
        //! \brief Save problem in file with given format (.xml, .json or .{dpomdp, posg, zsposg}).
        //! \param filename the file name
        void generateFile(std::string) const;

        /*!
    	 * \fn std::ostream& operator<<(std::ostream&, const dpomdp&)
    	 * \brief print the dpomdp
    	 * \param std::ostream&
    	 * \param const dpomdp& the individual decision rule to be printed
    	 * \return std::ostream&
    	 *
    	 * This method should produce an output of this form:
    	 */
        friend std::ostream &operator<<(std::ostream &os, const POSG &model)
        {
            os << model.toXML();
            return os;
        }
    };

    typedef POSG POStochasticGame;
    typedef POSG PartObsStochasticGame;
    typedef POSG PartiallyObservableStochasticGame;
} // namespace sdm
