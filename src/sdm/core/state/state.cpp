#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{

    double State::product(const std::shared_ptr<AlphaVector> &alpha)
    {
        return alpha->getValueAt(this->toState(), nullptr);
    }

    double State::product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action)
    {
        return beta->getValueAt(this->toState(), nullptr, action);
    }

    Pair<std::shared_ptr<State>, double> State::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return {observation->toState(), mdp->getTransitionProbability(this->toState(), action, observation->toState(), t)};
    }

    double State::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    {
        return mdp->getReward(this->toState(), action, t);
    }

    std::shared_ptr<BeliefInterface> State::toBelief()
    {
        return std::static_pointer_cast<BeliefInterface>(this->getPointer());
    }

    std::shared_ptr<OccupancyStateInterface> State::toOccupancyState()
    {
        return std::dynamic_pointer_cast<OccupancyStateInterface>(this->getPointer());
    }

    std::shared_ptr<HistoryInterface> State::toHistory()
    {
        return std::static_pointer_cast<HistoryInterface>(this->getPointer());
    }

    std::shared_ptr<SerialState> State::toSerial()
    {
        return std::static_pointer_cast<SerialState>(this->getPointer());
    }

    TypeState State::getTypeState() const
    {
        return TypeState::STATE;
    }

    size_t State::hash(double precision) const
    {
        throw exception::Exception("Hash (i.e. size_t X::hash() const ) is not implemented for this class");
    }

    bool State::isEqual(const std::shared_ptr<State> &, double) const
    {
        throw exception::Exception("Equal Operator (i.e. bool X::operator==() const ) is not implemented for this class");
    }

} // namespace sdm
