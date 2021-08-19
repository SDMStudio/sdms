#include <sdm/utils/value_function/light_extensive_qvalue_function.hpp>

// #include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    LightExtensiveQValueFunction::LightExtensiveQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer)
        : QValueFunction(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->Psi = std::vector<psi>(this->horizon_ + 1, psi());
        this->num_states_ = 0;
    }

    LightExtensiveQValueFunction::LightExtensiveQValueFunction(number horizon, double learning_rate, double default_value) : LightExtensiveQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer<Joint<std::shared_ptr<HistoryInterface>>>>(default_value))
    {
        
    }

    void LightExtensiveQValueFunction::initialize()
    {

    }

    void LightExtensiveQValueFunction::initialize(double default_value, number t)
    {

    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> LightExtensiveQValueFunction::getQValuesAt(const HistoryJointHistoryPair &state, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double LightExtensiveQValueFunction::getQValueAt(const HistoryJointHistoryPair &state, const std::shared_ptr<Action> &action, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double LightExtensiveQValueFunction::getQValueAt(const std::shared_ptr<HistoryInterface> &o1, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t)
    {
        this->initializeIfNeeded(o1, t);
        return this->Psi[t].at(o1).getQValueAt(o, u, t);
    }

    double LightExtensiveQValueFunction::getQValueAt(const std::shared_ptr<HistoryInterface> &o1, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return this->getQValueAt(o1, o->getIndividualHistories(), u, t);
    }
    
    void LightExtensiveQValueFunction::updateQValueAt(const HistoryJointHistoryPair &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        throw sdm::exception::NotImplementedException();
    }

    //
    void LightExtensiveQValueFunction::updateQValueAt(const std::shared_ptr<HistoryInterface> &o1, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t, double delta)
    {
        this->Psi[t].at(o1).updateQValueAt(o, u, t, delta);
    }

    void LightExtensiveQValueFunction::updateQValueAt(const std::shared_ptr<HistoryInterface> &o1, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta)
    {
        this->updateQValueAt(o1, o->getIndividualHistories(), u, t, delta);
    }

    void LightExtensiveQValueFunction::updateQValueAt(const HistoryJointHistoryPair &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool LightExtensiveQValueFunction::isNotSeen(const HistoryJointHistoryPair &state, number t)
    {
        return false;
    }

    int LightExtensiveQValueFunction::getNumStates() const
    {
        return this->num_states_;
    }

    void LightExtensiveQValueFunction::initializeIfNeeded(const std::shared_ptr<HistoryInterface> &o1, number t)
    {
        if (this->Psi[t].find(o1) == this->Psi[t].end())
        {
            this->Psi[t].emplace(o1, TabularQValueFunction<Joint<std::shared_ptr<HistoryInterface>>>(0, learning_rate_, initializer_));
            this->Psi[t].at(o1).initialize(0);
            if (t < this->horizon_)
            {
                this->num_states_++;
            }
        }
    }

    std::string LightExtensiveQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<extensive_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->Psi.size() - 1; i++)
        {
            res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
            for (auto const& [o1, q] : this->Psi[i])
            {
                if (o1->getHorizon() < this->horizon_)
                {
                    res << "\t\t<O1-Q>" << std::endl;
                    tools::indentedOutput(res, o1->str().c_str(), 3);
                    res << std::endl;
                    tools::indentedOutput(res, q.str().c_str(), 3);
                    res << "\t\t</O1-Q>" << std::endl;
                }
            }
            res << "\t</timestep>" << std::endl;
        }
        res << "</extensive_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm