#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    template <typename TBelief, typename TAction, typename TObservation>
    BeliefMDP<TBelief, TAction, TObservation>::BeliefMDP(std::shared_ptr<DecPOMDP> underlying_pomdp) : pomdp_(underlying_pomdp)
    {
        for (typename TBelief::state_type s : this->pomdp_->getStateSpace().getAll())
        {
            this->istate_[s] = this->pomdp_->getStartDistrib()[s];
        }
        this->cstate_ = this->istate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief &BeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->istate_;
    }
    template <typename TBelief, typename TAction, typename TObservation>
    TBelief &BeliefMDP<TBelief, TAction, TObservation>::getState()
    {
        return this->cstate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(TBelief belief, TAction action, TObservation obs) const
    {
        TBelief nextBelief;
        double tmp;
        for (number nextState = 0; nextState < this->pomdp_->getNumStates(); nextState++)
        {
            tmp = 0;
            for (number s = 0; s < this->pomdp_->getNumStates(); s++)
            {
                tmp += this->pomdp_->getTransitionProba(s, action, nextState) * belief.at(s);
            }
            nextBelief[nextState] = this->pomdp_->getObservationProbability(action, obs, nextState) * tmp;
        }
        // Normalize the belief
        double sum = nextBelief.norm_1();
        for (number s_ = 0; s_ < this->pomdp_->getNumStates(); s_++)
        {
            nextBelief[s_] = nextBelief[s_] / sum;
        }
        return nextBelief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(TBelief belief, TAction action) const
    {
        // Select o* as in the paper
        number selected_o = 0;
        double max_o = 0, tmp;

        for (number o = 0; o < this->pomdp_->getObsSpace()[0]; o++)
        {
            tmp = this->getObservationProbability(action, o, belief);
            auto tau = this->nextState(belief, action, o);
            tmp *= this->do_excess(tau, d + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_o = o;
            }
        }
        return this->nextState(belief, action, selected_o);
    }



    template <typename TBelief, typename TAction, typename TObservation>
    auto BeliefMDP<TBelief, TAction, TObservation>::getActionSpace(TBelief belief)
    {
        return this->pomdp_->getActionSpace();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getReward(TBelief belief, TAction action) const
    {
        double r = 0;
        for (auto &state : this->pomdp_->getStateSpace().getAll())
        {
            r += belief[state] * this->pomdp_->getReward(state, action);
        }
        return r;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, TBelief belief, TAction action, int t) const
    {
        double exp_next_v = 0;
        for (auto &obs : this->pomdp_->getObsSpace().getAll())
        {
            exp_next_v += this->getObservationProbability(action, obs, belief) * value_function->getValueAt(this->nextState(belief, action, obs), t + 1);
        }
        return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(TAction action, TObservation obs, TBelief belief) const
    {
        double proba = 0;
        for (auto &s : this->pomdp_->getStateSpace().getAll())
        {
            proba += this->pomdp_->getObservationProbability(action, obs, s) * belief.at(s);
        }
        return proba;
    }

} // namespace sdm
