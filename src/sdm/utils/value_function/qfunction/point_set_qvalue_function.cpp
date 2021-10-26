// #include <sdm/utils/value_function/point_set_qvalue_function.hpp>
// // #include <sdm/utils/value_function/backup/backup_base.hpp>
// // #include <sdm/core/state/interface/belief_interface.hpp>
// // #include <sdm/core/state/interface/occupancy_state_interface.hpp>

// namespace sdm
// {
//     PointSetQValueFunction::PointSetQValueFunction( std::shared_ptr<ValueInitializer> initializer,  const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionSelectionInterface> &action_selection)
//         : ValueFunction(initializer, backup, action_selection), TabularQValueFunction(0,initializer)
//     {
//     }


//     PointSetQValueFunction::PointSetQValueFunction( double default_value, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionSelectionInterface> &action_selection)
//         : PointSetQValueFunction(std::make_shared<ValueInitializer>(default_value),backup,action_selection)
//     {
//     }

//     double PointSetQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
//     {
//         //On va vouloir évaluer le point en s,a pas prendre sa valeur
//         return this->evaluateAtQ(state,action,t).second;
//     }

//     double PointSetQValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
//     {

//         std::shared_ptr<Action> best_action;
//         double best_value = -std::numeric_limits<double>::max(), value_tmp;

//         for(const auto& action_AND_value : this->representation[t].at(state))
//         {
//             if(best_value < (value_tmp = this->getQValueAt(state,action_AND_value.first,t)))
//             {
//                 best_value = value_tmp;
//                 best_action = action_AND_value.first;
//             }
//         }

//         return best_value;
//     }

//     Pair<std::shared_ptr<State>, double> evaluateAtQ(const std::shared_ptr<State>& state,const std::shared_ptr<Action>& action, number t)
//     {

//     }

//     void PointSetQValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
//     {        
//         auto [best_action, tmp] = this-getGreedyActionAndValue(state, t);
//         this->updateQValueAt(state,best_action, t, tmp);
//     }

//     void TabularQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
//     {
//         auto h = this->isInfiniteHorizon() ? 0 : t;
//         this->representation[h][state][action] = delta;
//     }


//     std::string PointSetQValueFunction::str() const
//     {
//         std::ostringstream res;
//         res << "<point_set_qvalue_function horizon=\"" << ((ValueFunction::isInfiniteHorizon()) ? "inf" : std::to_string(ValueFunction::getHorizon())) << "\">" << std::endl;
//         for (sdm::size_t i = 0; i < this->representation.size(); i++)
//         {
//             res << "\t<timestep=\"" << ((ValueFunction::isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
//             for (auto state__actions_values : this->representation[i])
//             {
//                 res << "\t\t<state id=\"" << state__actions_values.first << "\">" << std::endl;
//                 tools::indentedOutput(res, state__actions_values.first->str().c_str(), 3);
//                 res << std::endl;
//                 res << "\t\t</state>" << std::endl;
//                 res << "\t\t<actions>" << std::endl;
//                 for (auto action_value : state__actions_values.second)
//                 {
//                     res << "\t\t\t<action id=\"" << action_value.first << "\" value=" << action_value.second << ">" << std::endl;
//                     tools::indentedOutput(res, action_value.first->str().c_str(), 4);
//                     res << std::endl << "\t\t\t</action>" << std::endl;
//                 }
//                 res << "\t\t</actions>" << std::endl;
//             }
//             res << "\t</timestep>" << std::endl;
//         }

//         res << "</point_set_qvalue_function>" << std::endl;
//         return res.str();
//     }

//     void PointSetQValueFunction::do_pruning(number t){}

//     std::vector<std::shared_ptr<State>> PointSetQValueFunction::getSupport(number t)
//     {
//         //Pas implémenter ???? A faire un truc dans MappedMatrix
//         throw sdm::exception::NotImplementedException(); 
//     }

//     Container PointSetQValueFunction::getRepresentation(number t)
//     {
//         return this->representation[this->isInfiniteHorizon() ? 0 : t];
//     }


    // template <class Hash, class KeyEqual>
    // Pair<std::shared_ptr<State>, double> PointSetQValueFunction::evaluate(const std::shared_ptr<State> &state_tmp, number t)
    // {
    //     assert(this->getInitFunction() != nullptr);
    //     assert(state_tmp->getTypeState() != TypeState::STATE);

    //     auto state = state_tmp->toBelief();

    //     double min_ext = 0;
    //     double v_ub_state = this->getInitFunction()->operator()(state, t);

    //     std::shared_ptr<State> argmin_ = state;

    //     // Go over all element in the support
    //     for (const auto &point_value : this->getRepresentation(t))
    //     {
    //         auto [point, v_kappa] = point_value;
    //         auto point_to_belief_interface = point->toBelief();

    //         double v_ub_kappa = this->getInitFunction()->operator()(point, t);

    //         double phi;

    //         switch (state->getTypeState())
    //         {
    //         case TypeState::BELIEF_STATE:
    //             phi = this->ratioBelief(state, point_to_belief_interface);
    //             break;
    //         case TypeState::OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         case TypeState::SERIAL_OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         default:
    //             throw sdm::exception::Exception("PointSetQValueFunction::evaluate not defined for this state!");
    //             break;
    //         }

    //         // determine the min ext
    //         double min_int = phi * (v_kappa - v_ub_kappa);
    //         if (min_int < min_ext)
    //         {
    //             min_ext = min_int;
    //             argmin_ = point_to_belief_interface;
    //         }
    //     }
    //     return std::make_pair(argmin_, v_ub_state + min_ext);
    // }

    // template <class Hash, class KeyEqual>
    // double PointSetQValueFunction::ratioBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<BeliefInterface> &point)
    // {
    //     // Determine the ratio for the specific case when the state is a belief
    //     double phi = 1.0;

    //     for (auto &support : point->getStates())
    //     {
    //         double v_int = (state->getProbability(support) / point->getProbability(support));
    //         // determine the min int
    //         if (v_int < phi)
    //         {
    //             phi = v_int;
    //         }
    //     }
    //     return phi;
    // }

    // template <class Hash, class KeyEqual>
    // double PointSetQValueFunction::ratioOccupancy(const std::shared_ptr<BeliefInterface> &state_tmp, const std::shared_ptr<BeliefInterface> &point_tmp, number t)
    // {
    //     // Determine the ratio for the specific case when the state is a Occupancy State
    //     double phi = 1.0;
    //     auto point = point_tmp->toOccupancyState();
    //     auto occupancy_state = state_tmp->toOccupancyState();

    //     // Go over all joint history
    //     for (auto &joint_history : point->getJointHistories())
    //     {
    //         // Go over all hidden state in the belief conditionning to the joitn history
    //         for (const auto &hidden_state : point->getBeliefAt(joint_history)->getStates())
    //         {
    //             double v_int = (occupancy_state->getProbability(joint_history, hidden_state) / point->getProbability(joint_history, hidden_state));
    //             // determine the min int
    //             if (v_int < phi)
    //             {
    //                 phi = v_int;
    //             }
    //         }
    //     }
    //     return phi;
    // }

    // Pair<std::shared_ptr<State>, double> PointSetQValueFunction::evaluate(const std::shared_ptr<State> &state_tmp, number t)
    // {
    //     assert(this->getInitFunction() != nullptr);
    //     assert(state_tmp->getTypeState() != TypeState::STATE);

    //     auto state = state_tmp->toBelief();

    //     // double min_ext = 0;
    //     double min_value = 0;
        
    //     if(this->getSupport(t).size() != 0)
    //     {
    //         min_value = std::numeric_limits<double>::max();
    //     }

    //     double v_ub_state = this->getInitFunction()->operator()(state, t);

    //     std::shared_ptr<State> argmin_ = state;

    //     // Go over all element in the support
    //     for (const auto &point_value : this->getRepresentation(t))
    //     {
    //         auto [point, v_kappa] = point_value;
    //         auto point_to_belief_interface = point->toBelief();

    //         double v_ub_kappa = this->getInitFunction()->operator()(point->toOccupancyState()->getOneStepUncompressedOccupancy(), t);

    //         double phi;

    //         switch (state->getTypeState())
    //         {
    //         case TypeState::BELIEF_STATE:
    //             phi = this->ratioBelief(state, point_to_belief_interface);
    //             break;
    //         case TypeState::OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         case TypeState::SERIAL_OCCUPANCY_STATE:
    //             phi = this->ratioOccupancy(state, point_to_belief_interface,t);
    //             break;
    //         default:
    //             throw sdm::exception::Exception("PointSetQValueFunction::evaluate not defined for this state!");
    //             break;
    //         }

    //         // determine the min ext
    //         double min_int = phi * (v_kappa - v_ub_kappa);
    //         if (min_int < min_value)
    //         {
    //             min_value = min_int;
    //             argmin_ = point_to_belief_interface;
    //         }
    //     }
    //     return std::make_pair(argmin_,v_ub_state+ min_value);
    // }

    // double PointSetQValueFunction::ratioBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<BeliefInterface> &point)
    // {
    //     // Determine the ratio for the specific case when the state is a belief
    //     double phi = 1.0;

    //     for (auto &support : point->getStates())
    //     {
    //         double v_int = (state->getProbability(support) / point->getProbability(support));
    //         // determine the min int
    //         if (v_int < phi)
    //         {
    //             phi = v_int;
    //         }
    //     }
    //     return phi;
    // }

    // double PointSetQValueFunction::ratioOccupancy(const std::shared_ptr<BeliefInterface> &state_tmp, const std::shared_ptr<BeliefInterface> &point_tmp, number t)
    // {
    //     // Determine the ratio for the specific case when the state is a Occupancy State
    //     // double phi = 1.0;
    //     double min_value = std::numeric_limits<double>::max();

    //     auto point = point_tmp->toOccupancyState()->getOneStepUncompressedOccupancy();
    //     auto occupancy_state = state_tmp->toOccupancyState()->getOneStepUncompressedOccupancy();

    //     // Go over all joint history
    //     for (auto &joint_history : point->getJointHistories())
    //     {
    //         // Go over all hidden state in the belief conditionning to the joitn history
    //         for (const auto &hidden_state : point->getBeliefAt(joint_history)->getStates())
    //         {
    //             double v_int = (occupancy_state->getProbability(joint_history, hidden_state) / point->getProbability(joint_history, hidden_state));
    //             // determine the min int
    //             if (min_value > v_int)
    //             {
    //                 min_value = v_int;
    //             }
    //         }
    //     }
    //     return min_value;
    // }
// } // namespace sdm