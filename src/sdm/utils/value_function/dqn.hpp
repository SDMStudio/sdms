// #pragma once

// #include <sdm/utils/nn/mlpnet.hpp>

// /**
//  * @brief Namespace grouping all tools required for sequential decision making.
//  * @namespace  sdm
//  */
// namespace sdm
// {
//     class DQN
//     {

//     public:

//         DQN(number input_dim, number inner_dim, number output_dim, double learning_rate = 0.01);

//         /**
//          * @brief Get the q-value at a state
//          * 
//          * @param state the state
//          * @return the action value vector 
//          */
//         std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t);

//         /**
//          * @brief Get the q-value given state and action
//          * 
//          * @param state the state
//          * @param action the action
//          * @return the q-value
//          */
//         double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

//         // double getValueAt(const std::shared_ptr<State> &state, number t);

//         // std::shared_ptr<Action> getBestAction(const std::shared_ptr<State> &state, number t = 0);

//         /**
//          * @brief Update the value at a given state
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

//         /**
//          * @brief Update the value at a given state (given a delta)
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta);

//         bool isNotSeen(const std::shared_ptr<State> &state, number t);

//         /**
//          * @brief Define this function in order to be able to display the value function
//          */
//         virtual std::string str() const;

//         friend std::ostream &operator<<(std::ostream &os, DQN &vf)
//         {
//             os << vf.str();
//             return os;
//         }

//     protected:
//         double learning_rate_;
//         std::shared_ptr<DNN> representation;
//     };

// } // namespace sdm