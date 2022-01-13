#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>
#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

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
