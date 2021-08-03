#include <sdm/utils/value_function/backup/extensive_qvalue_backup.hpp>

#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>



namespace sdm
{
    ExtensiveQValueBackup::ExtensiveQValueBackup()
    {

    }

    ExtensiveQValueBackup::ExtensiveQValueBackup(
        std::shared_ptr<ExperienceMemory> experience_memory, 
        std::shared_ptr<QValueFunction<OccupancyStateJointHistoryPair>> q_value_table, 
        std::shared_ptr<QValueFunction<OccupancyStateJointHistoryPair>> target_q_value_table, 
        double discount,
        std::shared_ptr<Space> action_space
    ) : experience_memory_(experience_memory), 
        q_value_table_(std::dynamic_pointer_cast<ExtensiveQValueFunction>(q_value_table)),
        target_q_value_table_(std::dynamic_pointer_cast<ExtensiveQValueFunction>(target_q_value_table)), 
        discount_(discount), 
        action_space_(std::static_pointer_cast<MultiDiscreteSpace>(action_space))
    {

    }

    ExtensiveQValueBackup::~ExtensiveQValueBackup()
    {

    }

    double ExtensiveQValueBackup::update(number t)
    {   
        auto [observation, a, reward, next_observation, next_a] = this->experience_memory_->sample(t)[0];

        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(observation)->first;
        auto next_s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(next_observation)->first;
        auto o = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(observation)->second;
        auto next_o = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(next_observation)->second;
        auto u = this->action_space_->getItemAddress(*std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(a)->act(o->getIndividualHistories().toJoint<State>()))->toJoint<Item>())->toAction();
        auto next_u = this->action_space_->getItemAddress(*std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(next_a)->act(next_o->getIndividualHistories().toJoint<State>()))->toJoint<Item>())->toAction();

        double q_value = this->q_value_table_->getQValueAt(s, o->getIndividualHistories(), u, t);
        double next_value = this->q_value_table_->getQValueAt(next_s, next_o->getIndividualHistories(), next_u, t + 1);
        double target_q_value = reward + this->discount_ * next_value;
        double delta = target_q_value - q_value;
        this->q_value_table_->updateQValueAt(s, o->getIndividualHistories(), u, t, delta);

        return delta;
    }

    std::shared_ptr<Action> ExtensiveQValueBackup::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "-------- ExtensiveQValueBackup::getGreedyAction() ---------" << std::endl;
        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto o = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;

        ///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 2 /////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 2 /////////////////////////////////" << std::endl;

        std::unordered_map<std::shared_ptr<Action>, std::shared_ptr<DeterministicDecisionRule>>  possible_a2s;
        {
            for (const auto & u1: *this->action_space_->get(0))
            {
                std::vector<std::shared_ptr<Item>> inputs;
                std::vector<std::shared_ptr<Item>> outputs;
                for (const auto& possible_o : s->getJointHistories())
                {
                    inputs.push_back(possible_o->getIndividualHistory(1));
                    auto q_values_u2 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
                    for (const auto &u2 : *this->action_space_->get(1))
                    {
                        auto tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                        tmp_u->push_back(u1->toAction());
                        tmp_u->push_back(u2->toAction());
                        auto u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
                        q_values_u2->setValueAt(u2->toAction(), this->q_value_table_->getQValueAt(s, possible_o->getIndividualHistories(), u, t));
                    }
                    auto greedy_u2 = q_values_u2->argmax();
                    outputs.push_back(greedy_u2);
                }
                auto possible_a2 = std::make_shared<DeterministicDecisionRule>(inputs, outputs);
                possible_a2s.emplace(u1->toAction(), possible_a2);
            }
        }

        ///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 1 /////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        // We can skip. In multiagent code it won't be skipped.

        ///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 1 //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        std::shared_ptr<DeterministicDecisionRule> a1;
        {
            std::vector<std::shared_ptr<Item>> inputs;
            std::vector<std::shared_ptr<Item>> outputs;

            inputs.push_back(o->getIndividualHistory(0));
            auto q_values_agent_u1 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
            for (const auto &u1 : *this->action_space_->get(0))
            {
                q_values_agent_u1->setValueAt(u1->toAction(), 0);
                for (const auto &possible_o : s->getJointHistories())
                {
                    std::shared_ptr<Joint<std::shared_ptr<Action>>> tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    tmp_u->push_back(u1->toAction());
                    auto a2 = possible_a2s.at(u1->toAction());
                    auto u2 = a2->act(possible_o->getIndividualHistory(1));
                    tmp_u->push_back(u2);
                    auto u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
                    q_values_agent_u1->addValueAt(u1->toAction(), this->q_value_table_->getQValueAt(s, possible_o->getIndividualHistories(), u, t) * s->getProbability(possible_o));
                }
            }
            auto greedy_u1 = q_values_agent_u1->argmax();
            outputs.push_back(greedy_u1->toItem());
            a1 = std::make_shared<DeterministicDecisionRule>(inputs, outputs);
        }


        ////////////////////////////////////////////// CREATE DECISION RULE OF AGENT 2 //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 2 /////////////////////////////////" << std::endl;

        std::shared_ptr<DeterministicDecisionRule> a2 = possible_a2s.at(a1->act(o->getIndividualHistory(0)));

        ////////////////////////////////////////////// CREATE JOINT DECISION RULE //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE JOINT DECISION RULE /////////////////////////////////" << std::endl;


        std::vector<std::shared_ptr<DeterministicDecisionRule>> a;
        a.push_back(a1);
        a.push_back(a2);

        return std::make_shared<JointDeterministicDecisionRule>(a);
    }

    double ExtensiveQValueBackup::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "ExtensiveQValueBackup::getValueAt()" << std::endl;
        std::shared_ptr<OccupancyStateInterface> s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        std::shared_ptr<DecisionRule> a =  this->getGreedyAction(state, t)->toDecisionRule();

        double value = 0;
        for (const auto& o : s->getJointHistories())
        {
            for (const auto& u: *this->action_space_)
            {
                // v += s(o) * a(u|o) * Q_s(o, u, t)
                value += s->getProbability(o) * a->getProbability(o->getIndividualHistories().toJoint<State>(), u->toAction()) * this->target_q_value_table_->getQValueAt(s, o->getIndividualHistories(), u->toAction(), t);
            }
        }
        return value;
    }

}