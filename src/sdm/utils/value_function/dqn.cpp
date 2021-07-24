// #include <sdm/utils/value_function/dqn.hpp>
// #include <iterator>

// namespace sdm
// {


//     DQN::DQN(number input_dim, number inner_dim, number output_dim, double learning_rate = 0.01)
//     {
//         this->representation = std::make_shared<DNN>(input_dim, inner_dim, output_dim);
//     }

//     void DQN::initialize()
//     {

//     }

//     void DQN::initialize(double default_value, number t)
//     {

//     }

//     std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> DQN::getQValuesAt(const std::shared_ptr<State> &state, number t)
//     {

//     }

//     double DQN::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
//     {
//         return this->getQValuesAt(state, t)->at(action);
//     }

//     void DQN::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
//     {

//     }

//     void DQN::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
//     {
//         throw sdm::exception::NotImplementedException();
//     }

//     bool DQN::isNotSeen(const std::shared_ptr<State> &state, number t)
//     {
//         return false;
//     }

//     std::string DQN::str() const
//     {
//         std::ostringstream res;
//         res << "<deep-q-network" << std::endl;
//         // for (sdm::size_t i = 0; i < this->representation.size(); i++)
//         // {
//         //     res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
//         //     for (auto state__actions_values : this->representation[i])
//         //     {
//         //         res << "\t\t<state id=\"" << state__actions_values.first << "\">" << std::endl;
//         //         tools::indentedOutput(res, state__actions_values.first->str().c_str(), 3);
//         //         res << std::endl;
//         //         res << "\t\t</state>" << std::endl;
//         //         res << "\t\t<actions>" << std::endl;
//         //         for (auto action_value : state__actions_values.second)
//         //         {
//         //             res << "\t\t\t<action id=\"" << action_value.first << "\" value=" << action_value.second << ">" << std::endl;
//         //             tools::indentedOutput(res, action_value.first->str().c_str(), 4);
//         //             res << std::endl << "\t\t\t</action>" << std::endl;
//         //         }
//         //         res << "\t\t</actions>" << std::endl;
//         //     }
//         //     res << "\t</timestep>" << std::endl;
//         // }

//         res << "</deep-q-network>" << std::endl;
//         return res.str();
//     }
// } // namespace sdm