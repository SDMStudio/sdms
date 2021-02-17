/**
 * @file posg.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This class provide a way to instantiate a <h1>POSG</h1>. 
 * @version 1.0
 * @date 24/11/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <vector>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>

#include <sdm/types.hpp>

#include <sdm/world/po_process.hpp>
#include <sdm/world/decision_process.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    /**
     * @class POSG
     * @brief Partially Observable Stochastic Game class.
     * 
     * This class can be used to instantiate any Finite POSG problem.
     * Example:
     *   Example 1
     * 
     * This is [an example](http://example.com/ "Title") inline link.
     * 
     * [This link](http://example.net/) has no title attribute.
     * 
     * Example 2
     * 
     * The FFT is a fast implementation of the discrete Fourier transform:
     * @f[ X(e^{j\omega } ) = x(n)e^{ - j\omega n} @f]
     * 
     * The discrete-time Fourier time-convolution property states that
     * @f{eqnarray*}
     * x(n) * y(n) \Leftrightarrow X(e^{j\omega } )Y(e^{j\omega } )\\
     * x(n) * y(n) \Leftrightarrow X(e^{j\omega } )Y(e^{j\omega } )\\
     * x(n) * y(n) \Leftrightarrow X(e^{j\omega } )Y(e^{j\omega } )\\
     * x(n) * y(n) \Leftrightarrow X(e^{j\omega } )Y(e^{j\omega } )\\
     * @f}
     * 
     * This is an amazing function! For example:
     *
     *     auto cosines = cos(angles);
     *
     * Comment explaining the second example.
     *
     *     auto cosines = cos(radians(angles));
     * 
     * 1.  Item1
     * 
     *     More text for item1
     * 
     * 2.  Item2
     * 
     *         Code block for item2
     * 
     * 
     * A `Hello` sentence.
     * 
     * - Item1 of list 1
     * - Item2 of list 1
     * 
     * 1. Item1 of list 2
     * 2. Item2 of list 2
     * 
     * *AstSalut*
     * 
     * **DoubleAstSalut**
     * 
     * _UnderSalut_
     * 
     * __DoubleUnderSalut__
     * 
     * 
     */

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TDistrib>
    class POSG : public DecisionProcess<TStateSpace, TActionSpace, TDistrib>, public POProcess<TStateSpace, TObsSpace, TDistrib>
    {
    protected:
        ObservationDynamics o_dynamics_;

        //! \brief Map (state, jaction) to probability of (next_state, next_observation) --> i.e. s_{t+1}, o_{t+1} ~ P(S_{t+1}, O_{t+1}  | S_t = s, A_t = a )
        std::unordered_map<TStateSpace::value_type, std::unordered_map<TActionSpace::value_type, std::discrete_distribution<std::size_t>>> dynamics_generator;

        //! \brief map integer representing joint state/observation to this couple (state, observation)
        std::unordered_map<number, std::pair<TStateSpace::value_type, TObsSpace::value_type>> encoding;

        //! \fn void setupDynamicsGenerator()
        //! \brief Setup de dynamics generator
        void setupDynamicsGenerator();

    public:

        POSG();

        POSG(POSG &posg);

        //! \brief ruct a POSG from a SG (i.e. build observation function as identity).
        //! \param stochastic_game stochastic game
        POSG(SG &stochastic_game);

        //! \brief ruct a POSG from a file describing the problem.
        //! \param filename name of the file describing the POSG
        POSG(const std::string &filename);
        
        //! \brief    Construct a POSG.
        //! \param    state_sp the state space
        //! \param    agent_sp the agent space
        //! \param    action_sp the multi action space
        //! \param    obs_sp the multi observation space
        //! \param    s_dyn state dynamics
        //! \param    o_dyn observation dynamics
        //! \param    rews reward functions for each agent
        //! \param    start_distrib start distribution (optional)
        POSG(const TStateSpace &, const DiscreteSpace<number> &, const TActionSpace &, const TObsSpace &,
             const StateDynamics &, const ObservationDynamics &, const std::vector<Reward> &, const Vector &);

        //! \fn std::tuple<double, observation, state> getDynamicsGenerator(state x, action a)
        //! \param state the current state
        //! \param jaction the joint action
        //! \return a tuple containing reward, next_osbservation and next_state
        std::tuple<std::vector<double>, observation, state> getDynamicsGenerator(number x, number a) ;

         ObservationDynamics &getObsDynamics() ;

        //! \fn       double getObservationProbability(number, number, number) 
        //! \brief    Getter for observation probability.
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \param    state a specific state
        //! \return   the observation probility
        double getObservationProbability(TActionSpace::value_type jaction, TObsSpace::value_type jobservation, TStateSpace::value_type state) const;

        //! \brief    Getter for observation probability.
        // double getObservationProbability(std::vector<number> jaction, std::vector<number> jobservation, number state) const;

        //! \fn        Matrix& getObservations(action)
        //! \brief    Getter for observation probabilities.
        //! \param    jaction a specific joint action (as single one)
        //! \return   the observation probabilities for the pre-defined action.
        const Matrix &getObservations(TActionSpace::value_type jaction) const;

        //! \fn        Matrix getObservations(action)
        //! \brief    Getter for observation probabilities.
        //! \param    jaction a specific joint action
        //! \return   a matrix of probability observations for the pre-defined action.
        // const Matrix &getObservations(std::vector<number> jaction) const;

        //! \fn       value getDynamics(state, action, observation, state) 
        //! \brief    Getter for dynamics probability.
        //! \param    cstate a specific state (timestep t)
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \param    nstate a specific state (timestep t+1)
        //! \return   the dynamics probability
        double getDynamics(TStateSpace::value_type cstate, TActionSpace::value_type jaction, TObsSpace::value_type jobservation, TStateSpace::value_type nstate) const;

        //! \fn        matrix getDynamics(action, observation) 
        //! \brief    Getter for transition matrix.
        //! \param    jaction a specific joint action
        //! \param    jobservation a specific joint observation
        //! \return   a matrix of dynamics probabilities
        const Matrix &getDynamics(TActionSpace::value_type jaction, TObsSpace::value_type jobservation) const;

        //! \fn std::string toStdFormat()
        //! \brief Encodes POSG class into a string (standard .posg or .dpomdp or .zsposg format).
        //! \return the resulting standard file format like string
        std::string toStdFormat() ;

        //! \fn std::string toXML()
        //! \brief Encodes POSG class into a string (XML format).
        //! \return the resulting XML like string
        std::string toXML() ;

        //! \fn std::string toJSON()
        //! \brief Encodes POSG class into a string (JSON format).
        //! \return the resulting JSON like string
        std::string toJSON() ;

        /**
         * @fn void generateFile(std::string)
         * @brief Save problem in file with given format (.xml, .json or .{dpomdp, posg, zsposg}).
         * 
         * @param filename the file name
         */
        void generateFile(std::string) ;

        friend std::ostream &operator<<(std::ostream &os,  POSG &model)
        {
            os << model.toXML();
            return os;
        }
    };

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TDistrib>
    using POStochasticGame =  POSG<TStateSpace, TActionSpace, TObsSpace, TDistrib>;

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TDistrib>
    using PartObsStochasticGame =  POSG<TStateSpace, TActionSpace, TObsSpace, TDistrib>;

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TDistrib>
    using PartiallyObservableStochasticGame =  POSG<TStateSpace, TActionSpace, TObsSpace, TDistrib>;
} // namespace sdm
