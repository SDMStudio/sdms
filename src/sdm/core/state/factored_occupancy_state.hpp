/**
 * @file state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for state class
 * @version 0.1
 * @date 11/12/2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <sdm/core/state.hpp>

namespace sdm
{
    /**
     * @brief A public interface for factored occupancy states, inheritates from state.
     *
     * This is a n-tuple of individual occupancy state, one for each agent involved in the game.
     */
    class FactoredOccupancyState : public State
    {

    private:
        /**
         * @brief n-tuple of individual occupancy states, one per agent
         */
        std::vector<std::shared_ptr<OccupancyState>> sub_states;

        /**
         * @brief model of the Network Distributed Markov Decision Process
         */
        std::shared_prt<NetworkedDistributedPOMDPInterface> ndpomdp;


    public:

        /**
         * @brief Default Constructor 
         * Constructs a new Factored Occupancy State object, by initialising the tuple of individual occupancy states, one per agent.
         * @param memory the length of the agents' memory
         * @param ndmdp the Network Distributed Markov Decision Process
         */
        FactoredOccupancyState(int memory, const std::shared_ptr<NetworkedDistributedPOMDPInterface> &ndpomdp);


        /**
         * @brief Constructor 
         * Constructs a new Factored Occupancy State object, by initialising the n-tuple of individual occupancy states, one per aegnt.
         */
        FactoredOccupancyState(const std::vector<std::shared_ptr<State>> &sub_states);


        /**
         * @brief Compute the dot product operation with an alpha vector.
         *
         * @param alpha the alpha vector
         * @return double the resulting product
         */
        virtual double product(const std::shared_ptr<AlphaVector> &alpha);

        /**
         * @brief Compute the dot product operation with a beta vector.
         *
         * @param beta the beta vector
         * @param action the action
         * @return double the resulting product
         */
        virtual double product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action);

        /**
         * @brief Compute the following state given an action a.
         * 
         * @param ndmdp the Markov Decision Process
         * @param action the action 
         * @param t the timestep
         * @return std::shared_ptr<State> the next state
         */
        virtual Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &ndmdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);

        /**
         * @brief Compute the reward of being in the current state and execute action a.
         * 
         * @param ndmdp the Markov Decision Process
         * @param action the action 
         * @param t the timestep
         * @return double the reward
         */
        virtual double getReward(const std::shared_ptr<MDPInterface> &ndmdp, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the hash of the state.
         *
         * The hash is used in tabular value functions in order to compare efficiently two states.
         * This function must be reimplemented in inherited classes.
         *
         * @return size_t the hash code
         */
        virtual size_t hash(double precision = -1) const;

        /**
         * @brief Check equality between two states.
         *
         * This function must be implemented in inherited classes.
         *
         * @param other the state to be compared to current state
         * @return true if states are equal
         * @return false if they are different
         */
        virtual bool isEqual(const std::shared_ptr<State> &other, double precision = -1) const;

        /**
         * @brief Get string representation of the state.
         */
        virtual std::string str() const;
    };

} // namespace sdm