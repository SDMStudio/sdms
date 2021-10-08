// #pragma once

// #include <sdm/utils/value_function/tabular_qvalue_function.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// /**
//  * @brief Namespace grouping all tools required for sequential decision making.
//  * @namespace  sdm
//  */
// namespace sdm
// {
//     class PieceWiseLinearConvexQValueFunction : public QValueFunction
//     {
//     protected:
//         using PSI = std::unordered_map<std::shared_ptr<OccupancyStateInterface>, TabularQValueFunction /*, HashSpecific, EqualSpecific*/>;

//         /**
//          * @brief The precision used to assign a representant to an occupancy states.
//          */
//         double granularity = 0.75;

//         /**
//          * @brief The datastructure storing the value function representation. 
//          * 
//          */
//         std::vector<PSI> representation;

//         /**
//          * @brief The initializer to use for this value function. 
//          * 
//          */
//         std::shared_ptr<QInitializer> initializer_;

//     public:

//         PieceWiseLinearConvexQValueFunction(number horizon, std::shared_ptr<QInitializer> initializer);
//         PieceWiseLinearConvexQValueFunction(number horizon = 0, double default_value = 0.);

//         /**
//          * @brief Initialize the value function 
//          */
//         void initialize();

//         /**
//          * @brief Initialize the value function with a default value
//          */
//         void initialize(double v, number t = 0);

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
//         double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

//         double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

//         /**
//          * @brief Update the value at a given state
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

//         /**
//          * @brief Update the value at a given state (given a delta)
//          */
//         void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta);

//         void updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &state, const std::shared_ptr<JointHistoryInterface> &history, const std::shared_ptr<Action> &action, number t, double delta);

//         int getNumStates() const;

//         /**
//          * @brief Define this function in order to be able to display the value function
//          */
//         virtual std::string str() const;

//         friend std::ostream &operator<<(std::ostream &os, PieceWiseLinearConvexQValueFunction &vf)
//         {
//             os << vf.str();
//             return os;
//         }
//     };

// } // namespace sdm