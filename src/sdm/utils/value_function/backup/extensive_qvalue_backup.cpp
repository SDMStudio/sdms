#include <sdm/utils/value_function/backup/extensive_qvalue_backup.hpp>

#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/jhistory_tree.hpp>



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

        auto s = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(observation)->first;
        auto next_s = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(next_observation)->first;
        auto o = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(observation)->second;
        auto next_o = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(next_observation)->second;
        auto u = this->action_space_->getItemAddress(*std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(a)->act(o->getIndividualHistories().toJoint<State>()))->toJoint<Item>())->toAction();
        auto next_u = this->action_space_->getItemAddress(*std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(next_a)->act(next_o->getIndividualHistories().toJoint<State>()))->toJoint<Item>())->toAction();

        double q_value = this->q_value_table_->getQValueAt(s, o, u, t);
        double next_value = this->q_value_table_->getQValueAt(next_s, next_o, next_u, t + 1);
        double target_q_value = reward + this->discount_ * next_value;
        double delta = target_q_value - q_value;
        this->q_value_table_->updateQValueAt(s, o, u, t, delta);

        return delta;
    }

    std::shared_ptr<Action> ExtensiveQValueBackup::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "-------- ExtensiveQValueBackup::getGreedyAction() ---------" << std::endl;
        auto s = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(state)->first;
        auto o = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(state)->second;
        return this->getGreedyAction(s, o, t);
    }

    std::shared_ptr<DeterministicDecisionRule> ExtensiveQValueBackup::get_a1(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<JointHistoryInterface>& o, const std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>>& a2s, number t)
    {
        auto a1 = this->initializeDecisionRule();
        auto q_values = this->initializeQValues();
        for (const auto &u1 : *this->action_space_->get(0))
        {
            q_values->setValueAt(u1, 0);
            for (const auto &possible_o : s->getJointHistories())
            {
                auto a2 = a2s.at(u1);
                auto possible_o2 = possible_o->getIndividualHistory(1);
                auto u2 = a2->act(possible_o2);
                auto u = this->constructJointAction(u1, u2);
                q_values->addValueAt(u1, this->q_value_table_->getQValueAt(s, possible_o, u, t) * s->getProbability(possible_o));
            }
        }
        auto greedy_u1 = q_values->argmax();
        a1->addCase(o->getIndividualHistory(0), greedy_u1->toAction());
        return a1;
    }

    std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>> ExtensiveQValueBackup::get_a2s(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<JointHistoryInterface>& o, number t)
    {
        auto a2s = this->initialize_a2s();
        for (const auto & u1: *this->action_space_->get(0))
        {
            auto a2 = this->initializeDecisionRule();
            for (const auto& possible_o : s->getJointHistories())
            {
                auto q_values = this->initializeQValues();
                for (const auto &u2 : *this->action_space_->get(1))
                {
                    auto u = this->constructJointAction(u1, u2);
                    q_values->setValueAt(u2, this->q_value_table_->getQValueAt(s, possible_o, u, t));
                }
                auto greedy_u2 = q_values->argmax();
                a2->addCase(possible_o->getIndividualHistory(1), greedy_u2->toAction());
            }
            a2s.emplace(u1, a2);
        }
        return a2s;
    }

    std::shared_ptr<Action> ExtensiveQValueBackup::getGreedyAction(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<JointHistoryInterface>& o, number t)
    {
        auto a2s = this->get_a2s(s, o, t);

        auto a1 = this->get_a1(s, o, a2s, t);

        auto o1 = o->getIndividualHistory(0);

        auto u1 = a1->act(o1);
        
        auto a2 = a2s.at(u1);

        return this->constructJointDecisionRule(a1, a2);
    }

    double ExtensiveQValueBackup::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "ExtensiveQValueBackup::getValueAt()" << std::endl;
        std::shared_ptr<OccupancyStateInterface> s = std::dynamic_pointer_cast<OccupancyStateJointHistoryPointerPair>(state)->first;
        std::shared_ptr<DecisionRule> a =  this->getGreedyAction(state, t)->toDecisionRule();

        double value = 0;
        for (const auto& o : s->getJointHistories())
        {
            for (const auto& u: *this->action_space_)
            {
                // v += s(o) * a(u|o) * Q_s(o, u, t)
                value += s->getProbability(o) * a->getProbability(o->getIndividualHistories().toJoint<State>(), u->toAction()) * this->target_q_value_table_->getQValueAt(s, o, u->toAction(), t);
            }
        }
        return value;
    }

    std::shared_ptr<Action> ExtensiveQValueBackup::constructJointAction(const std::shared_ptr<Item> &u1, const std::shared_ptr<Item> &u2)
    {
        std::shared_ptr<Joint<std::shared_ptr<Action>>> tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
        tmp_u->push_back(u1->toAction());
        tmp_u->push_back(u2->toAction());
        return this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
    }

    std::shared_ptr<Action> ExtensiveQValueBackup::constructJointDecisionRule(const std::shared_ptr<DeterministicDecisionRule> &a1, const std::shared_ptr<DeterministicDecisionRule> &a2)
    {
        return std::make_shared<JointDeterministicDecisionRule>(std::vector<std::shared_ptr<DeterministicDecisionRule>>{a1, a2});
    }

    std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>> ExtensiveQValueBackup::initialize_a2s()
    {
        return std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>>();
    }

    std::shared_ptr<MappedVector<std::shared_ptr<Item>, double>> ExtensiveQValueBackup::initializeQValues()
    {
        return std::make_shared<MappedVector<std::shared_ptr<Item>, double>>();
    }

    std::shared_ptr<DeterministicDecisionRule> ExtensiveQValueBackup::initializeDecisionRule()
    {
        return std::make_shared<DeterministicDecisionRule>();
    }

}