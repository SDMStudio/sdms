#include <sdm/utils/value_function/pwlc_qvalue_function.hpp>

namespace sdm
{
    PieceWiseLinearConvexQValueFunction::PieceWiseLinearConvexQValueFunction(number horizon, std::shared_ptr<QInitializer> initializer)
        : QValueFunction(horizon), initializer_(initializer)
    {
        this->representation = std::vector<PSI>(this->isInfiniteHorizon() ? 1 : this->getHorizon() + 1, PSI());
    }

    PieceWiseLinearConvexQValueFunction::PieceWiseLinearConvexQValueFunction(number horizon, double default_value) : PieceWiseLinearConvexQValueFunction(horizon, std::make_shared<ValueInitializer>(default_value))
    {
    }

    void PieceWiseLinearConvexQValueFunction::initialize()
    {
    }

    void PieceWiseLinearConvexQValueFunction::initialize(double default_value, number t)
    {
    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> PieceWiseLinearConvexQValueFunction::getQValuesAt(const std::shared_ptr<State> &state, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double PieceWiseLinearConvexQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double PieceWiseLinearConvexQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return this->representation[h].at(s).getQValueAt(o, u, t);
    }

    void PieceWiseLinearConvexQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        for (auto o : state->toOccupancyState()->getJointHistories())
        {
            auto u = action->toDecisionRule()->act(o);
            this->representation[h].at(state->toOccupancyState()).updateQValueAt(o, u, t, delta * state->toOccupancyState()->getProbability(o));
        }
    }

    void PieceWiseLinearConvexQValueFunction::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double value)
    {
        throw sdm::exception::NotImplementedException();
    }

    void PieceWiseLinearConvexQValueFunction::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    int PieceWiseLinearConvexQValueFunction::getNumStates() const
    {
        // return this->num_states_;
        return 0;
    }

    std::string PieceWiseLinearConvexQValueFunction::str() const
    {
        std::ostringstream res;
        // res << "<hierarchical_qvalue_function_v2 horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        // for (sdm::size_t i = 0; i < this->representation.size(); i++)
        // {
        //     res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
        //     for (auto const& [s, q] : this->representation[i])
        //     {
        //         auto jh = *s->getJointHistories().begin();
        //         if (jh->getHorizon() < this->horizon_)
        //         {
        //             res << "\t\t<S-Q>" << std::endl;
        //             tools::indentedOutput(res, s->str().c_str(), 3);
        //             res << std::endl;
        //             tools::indentedOutput(res, q.str().c_str(), 3);
        //             res << "\t\t</S-Q>" << std::endl;
        //         }
        //     }
        //     res << "\t</timestep>" << std::endl;
        // }
        // res << "</hierarchical_qvalue_function_v2>" << std::endl;
        return res.str();
    }
} // namespace sdm