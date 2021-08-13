#include <sdm/utils/value_function/extensive_qvalue_function.hpp>

// #include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    ExtensiveQValueFunction::ExtensiveQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer)
        : QValueFunction(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->Psi = std::vector<psi>(this->horizon_ + 1, psi());
        this->num_states_ = 0;
    }

    ExtensiveQValueFunction::ExtensiveQValueFunction(number horizon, double learning_rate, double default_value) : ExtensiveQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer<Joint<std::shared_ptr<HistoryInterface>>>>(default_value))
    {
        
    }

    void ExtensiveQValueFunction::initialize()
    {

    }

    void ExtensiveQValueFunction::initialize(double default_value, number t)
    {

    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> ExtensiveQValueFunction::getQValuesAt(const OccupancyStateJointHistoryPair &state, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double ExtensiveQValueFunction::getQValueAt(const OccupancyStateJointHistoryPair &state, const std::shared_ptr<Action> &action, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double ExtensiveQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t)
    {
        this->initializeIfNeeded(s, t);
        return this->Psi[t].at(s).getQValueAt(o, u, t);
    }

    double ExtensiveQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        return this->getQValueAt(s, o->getIndividualHistories(), u, t);
    }
    
    void ExtensiveQValueFunction::updateQValueAt(const OccupancyStateJointHistoryPair &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        throw sdm::exception::NotImplementedException();
    }

    //
    void ExtensiveQValueFunction::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t, double delta)
    {
        this->Psi[t].at(s).updateQValueAt(o, u, t, delta);
    }

    void ExtensiveQValueFunction::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta)
    {
        this->updateQValueAt(s, o->getIndividualHistories(), u, t, delta);
    }

    void ExtensiveQValueFunction::updateQValueAt(const OccupancyStateJointHistoryPair &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool ExtensiveQValueFunction::isNotSeen(const OccupancyStateJointHistoryPair &state, number t)
    {
        return false;
    }

    int ExtensiveQValueFunction::getNumStates() const
    {
        return this->num_states_;
    }

    void ExtensiveQValueFunction::initializeIfNeeded(const std::shared_ptr<OccupancyStateInterface> &s, number t)
    {
        if (this->Psi[t].find(s) == this->Psi[t].end())
        {
            this->Psi[t].emplace(s, TabularQValueFunction<Joint<std::shared_ptr<HistoryInterface>>>(0, learning_rate_, initializer_));
            this->Psi[t].at(s).initialize(0);
            if (t < this->horizon_)
            {
                this->num_states_++;
            }
        }
    }

    std::string ExtensiveQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<extensive_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->Psi.size() - 1; i++)
        {
            res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
            for (auto const& [s, q] : this->Psi[i])
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
        res << "</extensive_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm