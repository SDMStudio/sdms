#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>

#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{
    HierarchicalQValueFunction::HierarchicalQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer, std::shared_ptr<Space> action_space)
        : QValueFunction(horizon), learning_rate_(learning_rate), initializer_(initializer), action_space_(action_space)
    {
        this->representation = std::unordered_map<std::shared_ptr<State>, Container>();
    }

    HierarchicalQValueFunction::HierarchicalQValueFunction(number horizon, double learning_rate, double default_value, std::shared_ptr<Space> action_space) : HierarchicalQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer>(default_value), action_space)
    {
    }

    void HierarchicalQValueFunction::initialize()
    {

    }

    void HierarchicalQValueFunction::initialize(double default_value, number t)
    {

    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> HierarchicalQValueFunction::getQValuesAt(const std::shared_ptr<State> &state, number t)
    {
        // // std::cout << "HierarchicalQValueFunction::getQValuesAt" << std::endl;
        // this->initializeIfNeeded(state);
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        // auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        // return this->representation[phos].getQValuesAt(jh, t);
    }

    double HierarchicalQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValuesAt(state, t)->at(action);
    }

    double HierarchicalQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &state, const std::shared_ptr<JointHistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValuesAt(std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryPair>(std::make_pair(state, history)), t)->at(action);
    }

    double HierarchicalQValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->getQValuesAt(state, t)->max();
    }

    std::shared_ptr<Action> HierarchicalQValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        // // FOR NOW N = 2 ! So there are only agents 1 and 2.
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;



        // // Accessible states for the b of agent 2.
        // std::vector<JointHistoryJointActionPair> acc_states_b2;
        // // Actions for each of these acc_states_b2.
        // std::vector<std::shared_ptr<Action>> n_actions_b2;

        // for (const auto &joint_history : phos->getFullyUncompressedOccupancy()->getJointHistories())
        // {
        //     std::shared_ptr<JointHistoryInterface> individual_hierarchical_history = std::make_shared<JointHistoryTree>();
        //     for (int lower_ranked_agent = 0; lower_ranked_agent < this->num_agents_; lower_ranked_agent++)
        //     {
        //         std::shared_ptr<HistoryInterface> individual_history = joint_history->getIndividualHistory(lower_ranked_agent);
        //         individual_hierarchical_history->addIndividualHistory(individual_history);
        //     }
        //     // individual_hierarchical_history ..........
        //     for (const auto &lower_ranked_agents_joint_action : lower_ranked_agents_joint_actions_for_each_agent_[1])
        //     {
        //         acc_states_b2.push_back(std::make_pair(individual_hierarchical_history, lower_ranked_agents_joint_action));
        //         std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> q_values_agent_2 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
        //         for (const auto &u2 : *std::static_pointer_cast<MultiDiscreteSpace>(this->action_space_)->get(1))
        //         {
        //             q_values_agent_2->setValueAt(u2->toAction(), 0);
        //             auto u1 = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(lower_ranked_agents_joint_action)->get(0);
        //             std::shared_ptr<Joint<std::shared_ptr<Action>>> u = std::make_shared<Joint<std::shared_ptr<Action>>>();
        //             u->push_back(u1);
        //             u->push_back(u2->toAction());
        //             auto u_ = std::static_pointer_cast<MultiDiscreteSpace>(this->action_space_)->getItemAddress(*u->toJoint<Item>())->toAction();
        //             q_values_agent_2->setValueAt(u2->toAction(), q_values_agent_2->getValueAt(u2->toAction()) + this->getQValueAt(phos, individual_hierarchical_history, u_, t));
        //         }



        //         n_actions_b2.push_back(q_values_agent_2->argmax());
        //     }
        // }

        // std::shared_ptr<DecisionRule> b2 = std::make_shared<DeterministicDecisionRule>(acc_states_b2, n_actions_b2);



        // // Accessible states for the b of agent 1.
        // std::vector<JointHistoryJointActionPair> acc_states_b1;
        // // Actions for each of these acc_states_b1.
        // std::vector<std::shared_ptr<Action>> n_actions_b1;



        // return this->getQValuesAt(state, t)->argmax();
    }

    void HierarchicalQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        // auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        // this->representation[phos].updateQValueAt(jh, action, t, delta);
    }

    void HierarchicalQValueFunction::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool HierarchicalQValueFunction::isNotSeen(const std::shared_ptr<State> &state, number t)
    {
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        // auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        // return ((this->representation.find(phos) == this->representation.end()) && (this->representation[phos].isNotSeen(jh, t)));
    }

    void HierarchicalQValueFunction::initializeIfNeeded(const std::shared_ptr<State> &state)
    {   
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        // if (this->representation.find(phos) == this->representation.end())
        // {   
        //     this->representation.emplace(phos, Container(this->horizon_, learning_rate_, initializer_));
        //     // this->representation.at(phos).initialize();
        //     this->representation.at(phos).initialize(0);
        //     // if (phos.getHorizon() < this->horizon_)
        //     // {
        //     //     this->numberOfPrivateOccupancyStates_++;
        //     // }
        // }
    }


    std::shared_ptr<Action> HierarchicalQValueFunction::getJointAction(const std::shared_ptr<Action> &joint_decision_rule, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &ostate, number t)
    {
        // auto joint_labels = ostate->toOccupancyState()->getJointLabels(joint_history->getIndividualHistories()).toJoint<State>();
        // auto joint_hierarchical_labels = this->getJointHierarchicalHistory(joint_labels, ostate, t);
        // auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(joint_decision_rule)->act(joint_hierarchical_labels);
        // auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);
        // // Get the adress of the joint action object from the space of available joint action object.
        // auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(action_space_)->getItemAddress(*joint_action->toJoint<Item>());
        // return joint_action_address->toAction();
    }

    std::shared_ptr<State> HierarchicalQValueFunction::getJointHierarchicalHistory(const std::shared_ptr<State> &joint_labels, const std::shared_ptr<State> &ostate, number t) const
    {
        // // This is the reversed version of what we want, that is Joint Hierarchical Labels, that is Hierarchical Labels for each agent.
        // // Each Hierarchical Label contains Labels for agents between agent I and agent N.
        // std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels_reversed = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // // This is what we will use to record Labels of each agent, starting with agent N until agent 1. This is why it's reversed.
        // std::shared_ptr<JointHistoryInterface> individual_hierarchical_label_reversed = std::make_shared<JointHistoryTree>();
        // // For agent from N-1 till 0 (N till 1):
        // for(int agent = this->num_agents_ - 1; agent >= 0; agent--)
        // {
        //     // Push agent I's Label.
        //     auto individual_label = std::dynamic_pointer_cast<Joint<std::shared_ptr<State>>>(joint_labels)->get(agent);
        //     individual_hierarchical_label_reversed->addIndividualHistory(individual_label->toHistory());
        //     // This will be in the correct order, that is Labels for agent I till N.
        //     std::shared_ptr<JointHistoryInterface> individual_hierarchical_label = std::make_shared<JointHistoryTree>();
        //     //
        //     for(int i = std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label_reversed)->getNumAgents() - 1; i >= 0; i--)
        //     {
        //         individual_hierarchical_label->addIndividualHistory(individual_hierarchical_label_reversed->getIndividualHistory(i));
        //     }
        //     //
        //     for (const std::shared_ptr<JointHistoryInterface>& individual_hierarchical_history: ostate->toOccupancyState()->getIndividualHierarchicalHistoryVectorFor(t, agent))
        //     {
        //         if (*std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_history) == *std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label))
        //         {
        //             individual_hierarchical_label = individual_hierarchical_history;
        //             break;
        //         }
        //     }
        //     // Push Hierarchical Label for agent I.
        //     joint_hierarchical_labels_reversed->push_back(individual_hierarchical_label);
        // }
        // // This will be in the correct order, that is Hierarchical Labels from agent 1 to N.
        // std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // // 
        // for (int i = joint_hierarchical_labels_reversed->getNumAgents() - 1; i >= 0; i--)
        // {
        //     joint_hierarchical_labels->push_back(joint_hierarchical_labels_reversed->at(i));
        // }
        // return joint_hierarchical_labels->toJoint<State>();
    }

    std::vector<std::vector<std::shared_ptr<Action>>> HierarchicalQValueFunction::getLowerRankedAgentsJointActionsForEachAgent()
    {

    }


    std::string HierarchicalQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<hierarchical_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        // for (sdm::size_t i = 0; i < this->representation.size(); i++)
        // {
        //     res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
        //     for (auto state__actions_values : this->representation[i])
        //     {
        //         res << "\t\t<state id=\"" << state__actions_values.first << "\">" << std::endl;
        //         tools::indentedOutput(res, state__actions_values.first->str().c_str(), 3);
        //         res << std::endl;
        //         res << "\t\t</state>" << std::endl;
        //         res << "\t\t<actions>" << std::endl;
        //         for (auto action_value : state__actions_values.second)
        //         {
        //             res << "\t\t\t<action id=\"" << action_value.first << "\" value=" << action_value.second << ">" << std::endl;
        //             tools::indentedOutput(res, action_value.first->str().c_str(), 4);
        //             res << "\t\t\t</action>" << std::endl;
        //         }
        //         res << "\t\t</actions>" << std::endl;
        //     }
        //     res << "\t</timestep>" << std::endl;
        // }

        res << "</hierarchical_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm