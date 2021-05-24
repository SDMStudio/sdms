#include <sdm/world/base/base_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number) : dpomdp_(underlying_dpomdp)
    {
    }

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP(std::string underlying_dpomdp, number hist_length) : BaseOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::reset()
    {
        // Reset the joint history to initial value
        this->current_history_ = this->initial_history_;

        // Reset the occupancy state
        *this->current_state_ = *this->initial_state_;

        // Reset the underlying DecPOMDP
        this->dpomdp_->reset();

        // Return the occupancy (which is the observation in BaseOccupancyMDP formalism)
        return *this->current_state_;
    }

    template <typename TState, typename TAction>
    bool BaseOccupancyMDP<TState, TAction>::isSerialized() const
    {
        return false;
    }

    template <typename TState, typename TAction>
    DiscreteDecPOMDP *BaseOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> BaseOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        using decision_rule_t = typename TAction::value_type;

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<TAction> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(TAction(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TAction>>(vect_j_dr);
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &ostate, const TAction &oaction, number t) const
    {
        TState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteMDP> BaseOccupancyMDP<TState, TAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState<>, number, number>> BaseOccupancyMDP<TState, TAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getDiscount(number)
    {
        return this->dpomdp_->getDiscount();
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->dpomdp_->getDiscount(), horizon);
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        return std::min(ub - lb, cost_so_far + this->dpomdp_->getDiscount() * ub - incumbent) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TState, typename TAction>
    TAction BaseOccupancyMDP<TState, TAction>::selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h)
    {
        return ub->getBestAction(s, h);
    }

} // namespace sdm
