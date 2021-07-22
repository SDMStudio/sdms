#include <sdm/utils/value_function/hierarchical_qvalue_function_v1.hpp>



namespace sdm
{
    HierarchicalQValueFunctionV1::HierarchicalQValueFunctionV1(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer)
        : QValueFunction(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
        this->num_states_ = 0;
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
        throw sdm::exception::NotImplementedException();
    }

    double HierarchicalQValueFunctionV1::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double HierarchicalQValueFunctionV1::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t)
    {
        this->initializeIfNeeded(ostate, t);
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return this->representation[h].at(ostate).getQValueAt(joint_history, action, t);
    }

    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        throw sdm::exception::NotImplementedException();
    }

    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return this->representation[h].at(ostate).updateQValueAt(joint_history, action, t, delta);
    }

    void HierarchicalQValueFunctionV1::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool HierarchicalQValueFunctionV1::isNotSeen(const std::shared_ptr<State> &state, number t)
    {
        return false;
    }

    int HierarchicalQValueFunctionV1::getNumStates() const
    {
        return this->num_states_;
    }

    void HierarchicalQValueFunctionV1::initializeIfNeeded(const std::shared_ptr<State> &state, number t)
    {   
        // std::cout << "HierarchicalQValueFunctionV1::initializeIfNeeded()" << std::endl;
        auto s = state->toOccupancyState();
        auto h = this->isInfiniteHorizon() ? 0 : t;
        if (this->representation[h].find(s) == this->representation[h].end())
        {   
            this->representation[h].emplace(s, TabularQValueFunction(0, learning_rate_, initializer_));
            // this->representation.at(s).initialize();
            this->representation[h].at(s).initialize(0);
            this->num_states_++;
        }
    }

    std::string HierarchicalQValueFunctionV1::str() const
    {
        std::ostringstream res;
        res << "<hierarchical_qvalue_function_v1 horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
            for (auto const& [s, q] : this->representation[i])
            {
                auto jh = *s->getJointHistories().begin();
                if (jh->getHorizon() < this->horizon_)
                {
                    res << "\t\t<S-Q>" << std::endl;
                    tools::indentedOutput(res, s->str().c_str(), 3);
                    res << std::endl;
                    tools::indentedOutput(res, q.str().c_str(), 3);
                    res << "\t\t</S-Q>" << std::endl;
                }
            }
            res << "\t</timestep>" << std::endl;
        }
        res << "</hierarchical_qvalue_function_v1>" << std::endl;
        return res.str();
    }
} // namespace sdm