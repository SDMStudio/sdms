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

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/observation/observation.hpp>

namespace sdm
{
    class BeliefInterface;
    class OccupancyStateInterface;
    class HistoryInterface;
    class JointHistoryInterface;
    class SerialState;
    class SerialOccupancyInterface;
    class AlphaVector;
    class BetaVector;
    class MDPInterface;

    template <class BaseClass, class MyClass>
    std::shared_ptr<BaseClass> to(std::shared_ptr<MyClass> aPtr)
    {
        return std::dynamic_pointer_cast<BaseClass>(aPtr);
    }

    template <class MyClass>
    std::shared_ptr<BeliefInterface> toBelief(std::shared_ptr<MyClass> aPtr)
    {
        return sdm::to<BeliefInterface>(aPtr);
    }
    /**
     * @brief A public interface for states.
     *
     * Any class inheriting from this interface will be considered as generic state for algorithms.
     * Consider sections [ Theoritical Background](/tutorials/theory.html) and [ Algorithms](/tutorials/algorithms/) for more information.
     *
     */
    class State : public Observation
    {
    public:
        virtual ~State() {}

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
         * @param mdp the Markov Decision Process
         * @param action the action 
         * @param t the timestep
         * @return std::shared_ptr<State> the next state
         */
        virtual Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);

        /**
         * @brief Compute the reward of being in the current state and execute action a.
         * 
         * @param mdp the Markov Decision Process
         * @param action the action 
         * @param t the timestep
         * @return double the reward
         */
        virtual double getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t);

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
        virtual std::string str() const = 0;


        /** @brief Cast the state into a belief */
        virtual std::shared_ptr<BeliefInterface> toBelief();

        /** @brief Cast the state into an occupancy state */
        virtual std::shared_ptr<OccupancyStateInterface> toOccupancyState();

        /** @brief Cast the state into a history */
        virtual std::shared_ptr<HistoryInterface> toHistory();

        /** @brief Cast the state into a serial interface */
        virtual std::shared_ptr<SerialState> toSerial();
    };

    template <typename TItem_1, typename TItem_2, class SuperClass = Item>
    class ItemPair : public SuperClass, public Pair<TItem_1, TItem_2>
    {
    public:
        ItemPair(const TItem_1 &first_item, const TItem_2 &second_item) : Pair<TItem_1, TItem_2>(first_item, second_item) {}
        ItemPair(const Pair<TItem_1, TItem_2> &copy) : Pair<TItem_1, TItem_2>(copy) {}

        std::string str() const
        {
            std::ostringstream res;
            res << "{" << this->first->str() << "," << this->second->str() << "}";
            return res.str();
        }
    };

    template <typename TState_1, typename TState_2>
    using StatePair = ItemPair<TState_1, TState_2, State>;

    template <typename TAction_1, typename TAction_2>
    using ActionPair = ItemPair<TAction_1, TAction_2, Action>;

    using JointHistoryStatePair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<State>>;
    using JointHistoryBeliefPair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>;
} // namespace sdm