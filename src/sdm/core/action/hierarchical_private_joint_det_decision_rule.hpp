
#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{

    /**
   * @brief The joint deterministic decision rule class. This class is a function that maps joint generic states to joint generic actions. 
   * 
   * @tparam TState the state type
   * @tparam TAction the action type
   */
    template <typename TState, typename TAction>
    class HierarchicalPrivateJointDeterministicDecisionRule
        : public Pair<JointDeterministicDecisionRule<TState, TAction>, TAction>
    {
    public:
        using input_type = TState;
        using output = TAction;
        using output_type = typename Function<Joint<TState>, Joint<TAction>>::output_type;


        HierarchicalPrivateJointDeterministicDecisionRule();
        HierarchicalPrivateJointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions, std::vector<TAction> action_agent_n);
        HierarchicalPrivateJointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list, number action_agent_n = 0);

        Joint<TAction> act(const Joint<TState> &jobserv) const;

        void setProbability(const std::vector<TState> &, const std::vector<TAction> &);

        friend std::ostream &operator<<(std::ostream &os, const HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction> &phjdr)
        {
            os << "<private-hierarchical-joint-decision-rule n-agents = « " << phjdr.n_agents_ << " »>" << std::endl;
            int i = 0;
            for (auto phidr : phjdr.first)
            {   
                os << "\t" << "<private-hierarchical-decision-rule agent-id = « " << i++ << " »>" << std::endl;
                for (const auto &pair_s_a : phidr)
                {
                    os << "\t\t" << "<decision history = « (";
                    for(number j = 0; j < pair_s_a.first.size(); j++){
                        os << pair_s_a.first.at(j)->short_str();
                        if (j < pair_s_a.first.size() - 1){
                            os << ", ";
                        }
                    }
                    os << ") » action = « " << pair_s_a.second << " »/>" << std::endl;
                }
                os << "\t" << "</private-hierarchical-decision-rule>" << std::endl;
            }
            os << "\t" << "<private-hierarchical-decision-rule agent-id = « " << phjdr.n_agents_ - 1 << " »>" << std::endl;
            os << "\t\t" << "<action = « " << phjdr.second << " »/>" << std::endl;
            os << "\t" << "</private-hierarchical-decision-rule>" << std::endl;
            os << "</private-hierarchical-joint-decision-rule>" << std::endl;
            return os;
        }

        number getNumAgents() const;

    protected:
        number n_agents_;
    };

} // namespace sdm

#include <sdm/core/action/hierarchical_private_joint_det_decision_rule.tpp>
