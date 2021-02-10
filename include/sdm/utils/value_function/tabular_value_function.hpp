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
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/function.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

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
    class TabularValueFunction : public ValueFunction<TState, TAction, TValue>, public Function<TState, TValue>
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
        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
            : ValueFunction<TState, TAction, TValue>(problem, horizon), initializer_(initializer)
        {
            this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_, Container());
            this->initialize();
        }

        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon = 0, TValue default_value = 0.) : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
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
        TValue getValueAt(TState &state, int t = 0)
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
        TAction getBestAction(TState &state, int t = 0)
        {
            auto qvalues = this->getQValueAt(state, t);
            return qvalues->argmax();
        }

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(TState &state, TAction &action, int t = 0)
        {
            // implement bellman operator
            return this->getWorld()->getReward(state, action) + this->getWorld()->getDiscount() * this->getWorld()->getExpectedNextValue(this, state, action, t);
        }

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(TState &state, int t = 0)
        {
            std::shared_ptr<MappedVector<TAction, double>> q_s = std::make_shared<MappedVector<TAction, double>>();
            for (auto &a : this->getWorld()->getActionSpace(state).getAll())
            {
                (*q_s)[a] = this->getQValueAt(state, a, t);
            }
            return q_s;
        }

        TValue operator()(const TState &state)
        {
            return this->getValueAt(state);
        }

        void updateValueAt(TState &state, int t = 0)
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
            res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
            for (int i = 0; i < this->representation.size(); i++)
            {
                res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\""<<this->representation[i].getDefault()<<"\">" << std::endl;
                for (auto pair_st_val : this->representation[i])
                {
                    res << "\t\t<state id=\"" << pair_st_val.first << "\">" << std::endl;
                    res << "\t\t\t" << pair_st_val.second << std::endl;
                    res << "\t\t</state>" << std::endl;
                }
                res << "\t</value>" << std::endl;
            }

            res << "</tabular_value_function>" << std::endl;
            return res.str();
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
    // template <>
    // std::shared_ptr<VectorImpl<number, double>> MappedValueFunction<BeliefState, number, double>::getQValueAt(BeliefState &belief, int t)
    // {

    //     auto nextState = [](decltype(this->getWorld()) w, BeliefState st, number action, number o, number d) {
    //         BeliefState nextBelief;
    //         double tmp;
    //         for (number s_ = 0; s_ < w->getNumStates(); s_++)
    //         {
    //             tmp = 0;
    //             for (number s = 0; s < w->getNumStates(); s++)
    //             {
    //                 tmp += w->getTransitionProba(s, action, s_) * st.at(s);
    //             }
    //             nextBelief[s_] = w->getObservationProbability(action, o, s_) * tmp;
    //         }
    //         // Normalize the belief
    //         double sum = nextBelief.norm_1();
    //         for (number s_ = 0; s_ < w->getNumStates(); s_++)
    //         {
    //             nextBelief[s_] = nextBelief[s_] / sum;
    //         }

    //         return nextBelief;
    //     };

    //     number n_a = this->getWorld()->getNumActions(0);
    //     std::shared_ptr<DenseVector<number, double>> v = std::make_shared<DenseVector<number, double>>(n_a);
    //     BeliefState nextBelief(belief.size(), 0);

    //     for (number a = 0; a < n_a; a++)
    //     {
    //         // Compute R(b, a)
    //         double r_b_a = 0, bootstrap = 0;
    //         for (number state_ = 0; state_ < this->getWorld()->getNumStates(); state_++)
    //         {
    //             r_b_a += this->getWorld()->getReward(state_, a, 0) * belief.at(state_);
    //         }
    //         // Compute p(o | b, a)
    //         for (number o = 0; o < this->getWorld()->getNumObservations(0); o++)
    //         {
    //             double obs_proba = 0;
    //             for (number i = 0; i < this->getWorld()->getNumStates(); i++)
    //             {
    //                 obs_proba += this->getWorld()->getObservationProbability(a, o, i) * belief.at(i);
    //             }
    //             nextBelief = nextState(this->getWorld(), belief, a, o, t);
    //             bootstrap += obs_proba * this->getValueAt(nextBelief, t + 1);
    //         }

    //         (*v)[a] = r_b_a + this->getWorld()->getDiscount() * bootstrap;
    //     }
    //     return v;
    // }

    // /**
    //  * @brief Specialisation of getQValueAt for finite state and action
    //  *
    //  * @param state a specific state
    //  * @return A vector containing the q-values at one state.
    //  */
    // template <>
    // std::shared_ptr<VectorImpl<number, double>> MappedValueFunction<number, number, double>::getQValueAt(number &state, int t)
    // {
    //     double tmp;
    //     number n_a = this->getWorld()->getNumActions(0);
    //     std::shared_ptr<DenseVector<number, double>> v = std::make_shared<DenseVector<number, double>>(n_a);

    //     for (number a = 0; a < n_a; a++)
    //     {
    //         tmp = 0;
    //         for (number state_ = 0; state_ < this->getWorld()->getNumStates(); state_++)
    //         {
    //             tmp += this->getWorld()->getTransitionProba(state, a, state_) * this->getValueAt(state_, t + 1);
    //         }
    //         (*v)[a] = this->getWorld()->getReward(state, a, 0) + this->getWorld()->getDiscount() * tmp;
    //     }
    //     return v;
    // }

} // namespace sdm
// #include <sdm/utils/value_function/tabular_value_function.tpp>
