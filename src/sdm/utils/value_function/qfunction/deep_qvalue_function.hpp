// #pragma once

// #include <torch/torch.h>
// #include <sdm/utils/nn/mlpnet.hpp>
// #include <sdm/utils/value_function/qvalue_function.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// /**
//  * @brief Namespace grouping all tools required for sequential decision making.
//  * @namespace  sdm
//  */
// namespace sdm
// {
//     /**
//      * @brief 
//      * 
//      *  Q(S,A, \theta) = \sum_{o,u} S(o) * A(u|o) * q(S, o,u; \theta) -- nous cherchons a implemente q(; \theta)
//      *
//      */
//     class DeepQValueFunction : public QValueFunction
//     {
//     protected:
//         std::vector<nn::MlpNet> qnetworks;

//         number nactions, nobservations, memory, horizon;

//     public:
//         DeepQValueFunction(number num_actions, number num_observations, number memory, number horizon);

//         /**
//          * @brief Initialize the value function 
//          */
//         void initialize();

//         double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

//         double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

//         /**
//          * @brief Update the value at a given state
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);
        
//         /**
//          * @brief Update the value at a given state (given a delta)
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double delta,number t);

//         void updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, double delta, number t);

//         /**
//          * @brief Define this function in order to be able to display the value function
//          */
//         virtual std::string str() const;

//         friend std::ostream &operator<<(std::ostream &os, DeepQValueFunction &vf)
//         {
//             os << vf.str();
//             return os;
//         }
//     };

// } // namespace sdm
