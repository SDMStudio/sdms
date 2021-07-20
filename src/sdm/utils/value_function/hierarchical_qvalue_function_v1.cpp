#include <sdm/utils/value_function/hierarchical_qvalue_function_v1.hpp>



namespace sdm
{
    HierarchicalQValueFunctionV1::HierarchicalQValueFunctionV1(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer)
        : QValueFunction(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->representation = std::unordered_map<std::shared_ptr<OccupancyStateInterface>, TabularQValueFunction>();
    }

    HierarchicalQValueFunctionV1::HierarchicalQValueFunctionV1(number horizon, double learning_rate, double default_value) : HierarchicalQValueFunctionV1(horizon, learning_rate, std::make_shared<ValueInitializer>(default_value))
    {
    }

    void HierarchicalQValueFunctionV1::initialize()
    {

    }

    void HierarchicalQValueFunctionV1::initialize(double default_value, number t)
    {

    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> HierarchicalQValueFunctionV1::getQValuesAt(const std::shared_ptr<State> &state, number t)
    {
        this->initializeIfNeeded(state);
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        return this->representation[phos].getQValuesAt(jh, t);
        throw sdm::exception::NotImplementedException();
    }

    double HierarchicalQValueFunctionV1::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // std::cout << "HierarchicalQValueFunctionV1::getQValueAt() A" << std::endl;
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        return this->getQValueAt(phos, jh, action, t);
    }

    double HierarchicalQValueFunctionV1::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t)
    {
        // std::cout << "HierarchicalQValueFunctionV1::getQValueAt() B" << std::endl;
        this->initializeIfNeeded(ostate);
        return this->representation[ostate].getQValueAt(joint_history, action, t);
    }

    // double HierarchicalQValueFunctionV1::getValueAt(const std::shared_ptr<State> &state, number t)
    // {
    //     std::shared_ptr<DecisionRule> decision_rule =  this->getBestAction(state, t)->toDecisionRule();
    //     std::shared_ptr<OccupancyStateInterface> s = state->toOccupancyState();
    //     double max_value = 0;
    //     for (const auto& each_s_it : this->representation)
    //     {
    //         double value = 0;
    //         for (const auto& o : s->getJointHistories())
    //         {
    //             for (const auto& u: *this->action_space_)
    //             {
    //                 auto ho = this->getJointHierarchicalHistory(o, each_s_it.first, t);
    //                 value += s->getProbability(o) * decision_rule->getProbability(ho, u->toAction()) * this->getQValueAt(each_s_it.first, o, u->toAction(), t);
    //             }
    //         }
    //         max_value = std::max(max_value, value);
    //     }
    //     return max_value;
    // }



    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        this->updateQValueAt(phos, jh, action, t, delta);
    }

    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t, double delta)
    {
        // ...
        // ...
        //
        this->representation[ostate].updateQValueAt(joint_history, action, t, delta);
    }

    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool HierarchicalQValueFunctionV1::isNotSeen(const std::shared_ptr<State> &state, number t)
    {
        // auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        // auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        // return ((this->representation.find(phos) == this->representation.end()) && (this->representation[phos].isNotSeen(jh, t)));
        return false;
    }

    void HierarchicalQValueFunctionV1::initializeIfNeeded(const std::shared_ptr<State> &state)
    {   
        // std::cout << "HierarchicalQValueFunctionV1::initializeIfNeeded()" << std::endl;
        auto phos = state->toOccupancyState();
        if (this->representation.find(phos) == this->representation.end())
        {   
            // auto jh = *phos->getJointHistories().begin();
            // if (jh->getHorizon() < 1)
            // {
            //     std::cout << ++num_of_ << std::endl;
            //     std::cout << "INITIALIZE " << phos << std::endl;
            // }
            this->representation.emplace(phos, TabularQValueFunction(this->horizon_, learning_rate_, initializer_));
            // std::cout << "aaa" << std::endl;
            // this->representation.at(phos).initialize();
            this->representation.at(phos).initialize(0);
            // std::cout << "aaa" << std::endl;
            // if (phos.getHorizon() < this->horizon_)
            // {
            //     this->numberOfPrivateOccupancyStates_++;
            // }
        }
        else
        {
            // auto jh = *phos->getJointHistories().begin();
            // if (jh->getHorizon() < 1)
            // {
            //     std::cout << num_of_ << std::endl;
            // }
        }
    }


    // std::shared_ptr<Action> HierarchicalQValueFunctionV1::getJointAction(const std::shared_ptr<Action> &joint_decision_rule, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &ostate, number t)
    // {
    //     auto joint_labels = ostate->toOccupancyState()->getJointLabels(joint_history->getIndividualHistories()).toJoint<State>();
    //     auto joint_hierarchical_labels = this->getJointHierarchicalHistory(joint_labels, ostate, t);
    //     auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(joint_decision_rule)->act(joint_hierarchical_labels);
    //     auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);
    //     // Get the adress of the joint action object from the space of available joint action object.
    //     auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(action_space_)->getItemAddress(*joint_action->toJoint<Item>());
    //     return joint_action_address->toAction();
    // }




    std::string HierarchicalQValueFunctionV1::str() const
    {
        std::ostringstream res;
        res << "<hierarchical_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (auto const& [s, q] : this->representation)
        {
            auto jh = *s->getJointHistories().begin();
            if (jh->getHorizon() < this->horizon_)
            {
                res << "\t<S-Q>" << std::endl;
                tools::indentedOutput(res, s->str().c_str(), 2);
                res << std::endl;
                tools::indentedOutput(res, q.str().c_str(), 2);
                res << "\t</S-Q>" << std::endl;
            }
        }
        res << "</hierarchical_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm