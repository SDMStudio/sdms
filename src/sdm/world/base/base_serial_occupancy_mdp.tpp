
#include <sdm/core/space/function_space.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    BaseSerializedOccupancyMDP<TState, TAction>::BaseSerializedOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    BaseSerializedOccupancyMDP<TState, TAction>::BaseSerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number ) : serialized_mpomdp_(std::make_shared<SerializedMPOMDP>(underlying_dpomdp))
    {
        std::cout<<"\n oui ?";
    }

    template <typename TState, typename TAction>
    BaseSerializedOccupancyMDP<TState, TAction>::BaseSerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : BaseSerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    SerializedMPOMDP *BaseSerializedOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->serialized_mpomdp_.get();
    }

    template <typename TState, typename TAction>
    TState BaseSerializedOccupancyMDP<TState, TAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> BaseSerializedOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        // Get the individual possible histories for the current agent (as vector)
        auto indiv_hist = ostate.getIndividualHistories(ag_id);

        std::vector<typename TState::jhistory_type::element_type::ihistory_type> v_inputs(indiv_hist.begin(), indiv_hist.end());

        // Generate all individual decision rules for agent 'ag_id' (the current agent)
        FunctionSpace<TAction> f_indiv_dr_space(v_inputs, this->serialized_mpomdp_->getActionSpace(ag_id)->getAll());

        // Now we can return a discrete space of all indiv decision rules
        return std::make_shared<DiscreteSpace<TAction>>(f_indiv_dr_space.getAll());
    }

    template <typename TState, typename TAction>
    TState BaseSerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double BaseSerializedOccupancyMDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &ostate, const TAction &oaction, number t) const
    {
        TState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP> BaseSerializedOccupancyMDP<TState, TAction>::toMDP()
    {
        return this->serialized_mpomdp_->toMDP();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedBeliefMDP<SerializedBeliefState,number,Joint<number>>> BaseSerializedOccupancyMDP<TState, TAction>::toBeliefMDP()
    {
        return this->serialized_mpomdp_->toBeliefMDP();
    }

    template <typename TState, typename TAction>
    bool BaseSerializedOccupancyMDP<TState, TAction>::isSerialized() const
    {
        return true;
    }

    template <typename TState, typename TAction>
    double BaseSerializedOccupancyMDP<TState, TAction>::getDiscount(number horizon)
    {
        return this->serialized_mpomdp_->getDiscount(horizon);
    }

    template <typename TState, typename TAction>
    double BaseSerializedOccupancyMDP<TState, TAction>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(), horizon / this->serialized_mpomdp_->getNumAgents());
    }

    template <typename TState, typename TAction>
    double BaseSerializedOccupancyMDP<TState, TAction>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        return std::min(ub - lb, cost_so_far + this->serialized_mpomdp_->getDiscount(horizon) * ub - incumbent) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TState, typename TAction>
    TAction BaseSerializedOccupancyMDP<TState, TAction>::selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h)
    {
        return ub->getBestAction(s, h);
    }
}