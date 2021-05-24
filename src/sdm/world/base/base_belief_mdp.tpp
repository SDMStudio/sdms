#include <sdm/world/base/base_belief_mdp.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TObservation>
    BaseBeliefMDP<TState, TAction, TObservation>::BaseBeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    BaseBeliefMDP<TBelief, TAction, TObservation>::BaseBeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp) : pomdp_(underlying_pomdp) {}

    template <typename TState, typename TAction, typename TObservation>
    BaseBeliefMDP<TState, TAction, TObservation>::BaseBeliefMDP(std::string underlying_dpomdp) : BaseBeliefMDP(std::make_shared<DiscretePOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BaseBeliefMDP<TBelief, TAction, TObservation>::reset()
    {
        this->current_state_ = this->initial_state_;
        this->pomdp_->reset();
        return this->current_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::tuple<TBelief, std::vector<double>, bool> BaseBeliefMDP<TBelief, TAction, TObservation>::step(TAction action)
    {
        auto [next_obs, rewards, done] = this->pomdp_->step(action);
        this->current_state_ = this->nextState(this->current_state_, action, next_obs);
        return std::make_tuple(this->current_state_, rewards, done);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    bool BaseBeliefMDP<TBelief, TAction, TObservation>::isSerialized() const
    {
        return false;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    DiscretePOMDP *BaseBeliefMDP<TBelief, TAction, TObservation>::getUnderlyingProblem()
    {
        return this->pomdp_.get();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BaseBeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->initial_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteSpace<TAction>> BaseBeliefMDP<TBelief, TAction, TObservation>::getActionSpaceAt(const TBelief &)
    {
        return this->pomdp_->getActionSpace();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BaseBeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(std::shared_ptr<ValueFunction<TBelief, TAction>> value_function, const TBelief &belief, const TAction &action, number t) const
    {
        double exp_next_v = 0;
        for (const TObservation &obs : this->pomdp_->getObsSpace()->getAll())
        {
            const auto &next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(belief, action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;

        // double exp_next_v = 0;
        // for (TObservation observation : this->pomdp_->getObsSpace()->getAll())
        // {
        //     auto next_belief = belief->expand(action, observation)
        //     exp_next_v += this->getObservationProbability(next_belief , action, observation, next_belief) * value_function->getValueAt(next_belief, t + 1);
        // }
        // return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteMDP> BaseBeliefMDP<TBelief, TAction, TObservation>::toMDP()
    {
        return this->pomdp_->toMDP();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<BaseBeliefMDP<TBelief, TAction, TObservation>> BaseBeliefMDP<TBelief, TAction, TObservation>::toBeliefMDP()
    {
        return this->shared_from_this();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BaseBeliefMDP<TBelief, TAction, TObservation>::getDiscount(number)
    {
        return this->pomdp_->getDiscount();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BaseBeliefMDP<TBelief, TAction, TObservation>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->pomdp_->getDiscount(), horizon);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BaseBeliefMDP<TBelief, TAction, TObservation>::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TAction BaseBeliefMDP<TBelief, TAction, TObservation>::selectNextAction(const std::shared_ptr<ValueFunction<TBelief, TAction>> &, const std::shared_ptr<ValueFunction<TBelief, TAction>> &ub, const TBelief &s, number h)
    {
        return ub->getBestAction(s, h);
    }

} // namespace sdm
