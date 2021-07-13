#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>

namespace sdm
{
    HierarchicalQValueFunction::HierarchicalQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer)
        : QValueFunction(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->representation = std::unordered_map<std::shared_ptr<State>, Container>();
    }

    HierarchicalQValueFunction::HierarchicalQValueFunction(number horizon, double learning_rate, double default_value) : HierarchicalQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer>(default_value))
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
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        return this->representation[phos].getQValuesAt(jh, t);
    }

    double HierarchicalQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValuesAt(state, t)->at(action);
    }

    double HierarchicalQValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->getQValuesAt(state, t)->max();
    }

    std::shared_ptr<Action> HierarchicalQValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        return this->getQValuesAt(state, t)->argmax();
    }

    void HierarchicalQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        this->representation[phos].updateQValueAt(jh, action, t, delta);
    }

    void HierarchicalQValueFunction::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool HierarchicalQValueFunction::isNotSeen(const std::shared_ptr<State> &state, number t)
    {
        auto phos = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->first;
        auto jh = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(state)->second;
        return ((this->representation.find(phos) == this->representation.end()) && (this->representation[phos].isNotSeen(jh, t)));
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