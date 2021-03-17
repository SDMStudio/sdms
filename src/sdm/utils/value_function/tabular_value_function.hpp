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
#include <sdm/utils/backup_operator/backup_operator.hpp>
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
    template <typename TState,
              typename TAction,
              typename TValue = double,
              template <typename TI, typename TV> class TBackupOperator = ClassicBellmanBackupOperator,
              template <typename TI, typename TV> class TStruct = MappedVector>
    class TabularValueFunction : public ValueFunction<TState, TAction, TValue>
    {
    protected:
        using Container = TStruct<TState, TValue>;
        using backup_operator_type = TBackupOperator<TState, TAction>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<Container> representation;

        /**
         * @brief The backup operator used in order to update this value function. 
         * 
         */
        backup_operator_type backup_op_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer<TState, TAction>> initializer_;

    public:
        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon, std::shared_ptr<Initializer<TState, TAction>> initializer);

        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon = 0, TValue default_value = 0.);

        /**
         * @brief Initialize the value function according using initializer.
         * 
         */
        void initialize();

        /**
         * @brief Set all values of the vector to a default value. 
         * 
         * @param default_value the default value 
         */
        void initialize(TValue default_value, int t = 0);

        /**
         * @brief Evaluate the value at a state
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(TState &state, int t = 0);

        void updateValueAt(TState &state, int t = 0);
        void updateValueAt(TState &state, int t, TValue target);

        std::string str();

        backup_operator_type getBackupOperator();

        friend std::ostream &operator<<(std::ostream &os, TabularValueFunction<TState, TAction> &vf)
        {
            os << vf.str();
            return os;
        }
    };

    template <typename TState, typename TAction, typename TValue = double>
    using MappedValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, MappedVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using SparseValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, SparseVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using DenseValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm
#include <sdm/utils/value_function/tabular_value_function.tpp>

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
