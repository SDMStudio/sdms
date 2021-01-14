/**
 * @file tabular_value_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Tabular value function are functions of state and action that use a vector representation. 
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <iostream>
#include <type_traits>
#include <map>

#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/incremental_value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/core/state/state.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Tabular value function are functions of state and action that use a vector representation to store the values. 
     * 
     * @tparam TState Type of the states
     * @tparam TAction Type of the states
     * @tparam TValue Type of the values (must be primitive type)
     * @tparam TStruct Type of vector container (MappedVector, DenseVector and SparseVector are common type) 
     */
    template <typename TState, typename TAction, typename TValue = double, template <typename TI, typename TV> class TStruct = MappedVector>
    class TabularValueFunction : public ValueFunction<TState, TAction, TValue>
    {
    protected:
        using Container = TStruct<TState, TValue>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<Container> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer<TState, TAction>> initializer_;

    public:
        TabularValueFunction(std::shared_ptr<POSG> problem, int horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
            : ValueFunction<TState, TAction, TValue>(problem, horizon), initializer_(initializer)
        {
            this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_, Container());
            this->initialize();
        }

        TabularValueFunction(std::shared_ptr<POSG> problem, int horizon = 1, TValue default_value = 0.) : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
        {
        }

        /**
         * @brief Initialize the value function according using initializer.
         * 
         */
        void initialize()
        {
            this->initializer_->init(this);
        }

        /**
         * @brief Set all values of the vector to a default value. 
         * 
         * @param default_value the default value 
         */
        void initialize(TValue default_value, int t = 0)
        {
            if (this->isInfiniteHorizon())
            {
                this->representation[0] = Container(default_value);
            }
            else
            {
                assert(t < this->getHorizon());
                this->representation[t] = Container(default_value);
            }
        }

        /**
         * @brief Evaluate the value at a state
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(const TState &state, int t = 0)
        {
            if (this->isInfiniteHorizon())
            {
                return this->representation[0].at(state);
            }
            else
            {
                return (t >= this->getHorizon()) ? 0 : this->representation[t].at(state);
            }
        }

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        TAction getBestAction(const TState &state, int t = 0)
        {
            return this->getQValueAt(state, t)->argmax();
        }

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(const TState &state, const TAction &action, int t = 0)
        {
            // implement bellman operator
        }

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TState &state, int t = 0);

        void updateValueAt(const TState &state, int t = 0)
        {
            if (this->isInfiniteHorizon())
            {
                this->representation[0][state] = this->getQValueAt(state, t)->max();
            }
            else
            {
                assert(t < this->horizon_);
                this->representation[t][state] = this->getQValueAt(state, t)->max();
            }
        }

        std::string str()
        {
            std::ostringstream res;
            for (int i = 0; i < this->representation.size(); i++)
            {
                res << "V(t=" << i << ") = " << this->representation[i].str() << "\n";
            }
            return res.str();
        }

        friend std::ostream &operator<<(std::ostream &os, const TabularValueFunction &vf)
        {
            os << vf.str();
            return os;
        }
    };

    template <typename TState, typename TAction, typename TValue = double>
    using MappedValueFunction = TabularValueFunction<TState, TAction, TValue, MappedVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using SparseValueFunction = TabularValueFunction<TState, TAction, TValue, SparseVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using DenseValueFunction = TabularValueFunction<TState, TAction, TValue, DenseVector>;

    /**
     * @brief Specialisation of getQValueAt for belief state
     * 
     * @param belief a specific belief
     * @return A vector containing the q-values at one belief. 
     */
    template <>
    std::shared_ptr<VectorImpl<number, double>> MappedValueFunction<BeliefState, number, double>::getQValueAt(const BeliefState &belief, int t)
    {
        number n_a = this->getWorld()->getNumActions(0);
        std::shared_ptr<DenseVector<number, double>> v = std::make_shared<DenseVector<number, double>>(n_a);
        BeliefState nextBelief(belief.size(), 0);

        for (number a = 0; a < n_a; a++)
        {
            // Compute R(b, a)
            double r_b_a, bootstrap = 0;
            for (number state_ = 0; state_ < this->getWorld()->getNumStates(); state_++)
            {
                r_b_a += this->getWorld()->getReward(state_, a, 0) * belief[state_];
            }

            // Compute p(o | b, a)
            for (number o = 0; o < this->getWorld()->getNumObservations(0); o++)
            {
                double obs_proba;
                for (number i = 0; i < this->getWorld()->getNumStates(); i++)
                {
                    obs_proba += this->getWorld()->getObservationProbability(a, o, i) * belief[i];
                    double p_b_next = 0;
                    for (number s_ = 0; s_ < this->getWorld()->getNumStates(); s_++)
                    {
                        p_b_next += this->getWorld()->getTransitionProba(s_, a, i);
                    }
                    nextBelief[i] = this->getWorld()->getObservationProbability(a, o, i) * p_b_next;
                }

                bootstrap += obs_proba * this->getValueAt(nextBelief, t + 1);
            }

            (*v)[a] = r_b_a + this->getWorld()->getDiscount() * bootstrap;
        }
        return v;
    }

    /**
     * @brief Specialisation of getQValueAt for finite state and action
     * 
     * @param state a specific state
     * @return A vector containing the q-values at one state. 
     */
    template <>
    std::shared_ptr<VectorImpl<number, double>> MappedValueFunction<number, number, double>::getQValueAt(const number &state, int t)
    {
        double tmp;
        number n_a = this->getWorld()->getNumActions(0);
        std::shared_ptr<DenseVector<number, double>> v = std::make_shared<DenseVector<number, double>>(n_a);

        for (number a = 0; a < n_a; a++)
        {
            tmp = 0;
            for (number state_ = 0; state_ < this->getWorld()->getNumStates(); state_++)
            {
                tmp += this->getWorld()->getTransitionProba(state, a, state_) * this->getValueAt(state_, t + 1);
            }
            (*v)[a] = this->getWorld()->getReward(state, a, 0) + this->getWorld()->getDiscount() * tmp;
        }
        return v;
    }

    /**
     * @brief Specialisation of getQValueAt for dense vector and finite state and action
     * 
     * @param state a specific state
     * @return A vector containing the q-values at one state. 
     */
    template <>
    std::shared_ptr<VectorImpl<number, double>> DenseValueFunction<number, number, double>::getQValueAt(const number &state, int t)
    {
        double tmp;
        number n_a = this->getWorld()->getNumActions(0);
        std::shared_ptr<DenseVector<number, double>> v = std::make_shared<DenseVector<number, double>>(n_a);

        for (number a = 0; a < n_a; a++)
        {
            tmp = 0;
            for (number state_ = 0; state_ < this->getWorld()->getNumStates(); state_++)
            {
                tmp += this->getWorld()->getTransitionProba(state, a, state_) * this->getValueAt(state_, t + 1);
            }
            (*v)[a] = this->getWorld()->getReward(state, a, 0) + this->getWorld()->getDiscount() * tmp;
        }
        return v;
    }

} // namespace sdm
// #include <sdm/utils/value_function/tabular_value_function.tpp>
