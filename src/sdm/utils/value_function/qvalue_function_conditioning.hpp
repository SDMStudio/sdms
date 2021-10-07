// /**
//  * @file value_function.hpp
//  * @author Jérôme ARJONILLA 
//  * @brief Defines the Qvalue function conditioning interface.
//  * @version 0.1
//  * @date 17/08/2021
//  * 
//  * @copyright Copyright (c) 2021
//  * 
//  */
// #pragma once
// #include <sdm/utils/value_function/qvalue_function.hpp>

// /**
//  * @brief Namespace grouping all tools required for sequential decision making.
//  * @namespace  sdm
//  */
// namespace sdm
// {
//     template <class TCondition, class TState>
//     class QValueFunctionConditioning : public QValueFunction<Pair<TCondition,TState>>
//     {
//     protected:
//         /**
//          * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
//          * 
//          */
//         // std::shared_ptr<BinaryFunction<TState, std::shared_ptr<Action>, number, double>> init_function_ = nullptr;

//     public:
//         using q = Pair<TCondition,TState>;

//         QValueFunctionConditioning();

//         /**
//          * @brief Construct a new Incremental Value Function object
//          * 
//          * @param problem 
//          * @param default_value 
//          */
//         QValueFunctionConditioning(number horizon);

//         virtual double getQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t) = 0;

//         virtual void updateQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t = 0) = 0;
        
//         virtual void updateQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t, double delta) = 0;

//         /**
//          * 
//          * @brief Get the q-value given state and action
//          * 
//          * @param state the state
//          * @param action the action
//          * @return the q-value
//          */
//         virtual double getQValueAt(const TInput &input, const std::shared_ptr<Action> &action, number t);

//         /**
//          * @brief Update the value at a given input
//          */
//         virtual void updateQValueAt(const TInput &input, const std::shared_ptr<Action> &action, number t = 0);

//         /**
//          * @brief Update the value at a given input (given a target)
//          */
//         virtual void updateQValueAt(const TInput &input, const std::shared_ptr<Action> &action, number t, double target);

//         /**
//          * @brief Get shared pointer on the current QValueFunctionConditioning<TCondition>
//          */
//         std::shared_ptr<QValueFunctionConditioning<TCondition,TState>> getptr();

//     };
// } // namespace sdm
// #include <sdm/utils/value_function/qvalue_function_conditioning.tpp>
