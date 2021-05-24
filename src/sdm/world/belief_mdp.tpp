#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    template <typename TBelief, typename TAction, typename TObservation>
    BeliefMDP<TBelief, TAction, TObservation>::BeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    BeliefMDP<TBelief, TAction, TObservation>::BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp) : BaseBeliefMDP<TBelief, TAction, TObservation>(underlying_pomdp)
    {
        // Set initial belief state
        this->initial_state_ = TBelief(this->pomdp_->getStateSpace()->getAll(), this->pomdp_->getStartDistrib().probabilities());
        this->current_state_ = this->initial_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    BeliefMDP<TBelief, TAction, TObservation>::BeliefMDP(std::string underlying_pomdp) : BeliefMDP(std::make_shared<DiscretePOMDP>(underlying_pomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {
        // Compute next coef belief (non normailized)
        TBelief weighted_next_belief = this->pomdp_->getObsDynamics()->getDynamics(action, obs).transpose() ^ belief;

        // Compute the coefficient of normalization (eta)
        double eta = weighted_next_belief.norm_1();

        return (1. / eta) * weighted_next_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        TBelief select_next_state;
        for (const auto &o : this->pomdp_->getObsSpace()->getAll())
        {
            tmp = this->getObservationProbability(belief, action, o, belief);
            const auto &tau = this->nextState(belief, action, o);
            tmp *= hsvi->do_excess(tau, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                select_next_state = tau;
            }
        }
        return select_next_state;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getReward(const TBelief &belief, const TAction &action) const
    {
        return (belief ^ this->pomdp_->getReward()->getReward(action));
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TBelief &, const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        return (this->pomdp_->getObsDynamics()->getDynamics(action, obs).transpose() ^ belief).norm_1();
    }

    // ##############################################################################################################################
    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefState Structure ###############################################
    // ##############################################################################################################################
    // ##############################################################################################################################

    template <typename TAction, typename TObservation>
    BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::BeliefMDP()
    {
    }

    template <typename TAction, typename TObservation>
    BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp) : BaseBeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>(underlying_pomdp)
    {
        BeliefState<> belief(this->pomdp_->getStateSpace()->getAll(), this->pomdp_->getStartDistrib().probabilities());
        this->initial_state_ = std::make_shared<typename BeliefStateGraph_p<TAction, TObservation>::element_type>(belief, this->pomdp_->getObsDynamics()->getDynamics());
        this->current_state_ = this->initial_state_;
    }

    template <typename TAction, typename TObservation>
    BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::BeliefMDP(std::string underlying_pomdp) : BeliefMDP(std::make_shared<DiscretePOMDP>(underlying_pomdp))
    {
    }

    template <typename TAction, typename TObservation>
    typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::nextState(const typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type &belief, const TAction &action, const TObservation &observation) const
    {
        return belief->expand(action, observation);
    }

    template <typename TAction, typename TObservation>
    typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::nextState(const typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type &belief, const TAction &action, number t, std::shared_ptr<HSVI<typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type, TAction>> hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type select_next_state;
        for (const auto &o : this->pomdp_->getObsSpace()->getAll())
        {
            tmp = this->getObservationProbability(belief, action, o, belief);
            const auto &tau = this->nextState(belief, action, o);
            tmp *= hsvi->do_excess(tau, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                select_next_state = tau;
            }
        }

        return select_next_state;
    }

    template <typename TAction, typename TObservation>
    double BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::getReward(const typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type &belief, const TAction &action) const
    {
        return (*belief ^ this->pomdp_->getReward()->getReward(action));
    }

    template <typename TAction, typename TObservation>
    double BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::getObservationProbability(const typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type &, const TAction &action, const TObservation &observation, const typename BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>::state_type &belief) const
    {
        return belief->getProbability(action, observation);
    }

} // namespace sdm
