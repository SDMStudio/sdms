#include <sdm/utils/value_function/belief_2_occupancy_vf.hpp>

namespace sdm
{

    template <typename TBelief, typename TOccupancyState>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TBelief, number>> pomdp_vf) : pomdp_vf_(pomdp_vf)
    {
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_solving_dpomdp>
    std::enable_if_t<is_solving_dpomdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        double value = 0;

        // $sum_{o_{\tau}} p(o_{\tau} \mid s_{\tau} v_{\tau}^{pomdp}\left( x_{\tau} \mid o_{\tau} \right))$
        for (const auto &joint_history : ostate.getJointHistories())
        {
            TBelief belief = ostate.createBeliefWeighted(joint_history);
            double sum = belief.norm_1();
            for (const auto &b_s : belief)
            {
                belief[b_s.first] = belief[b_s.first] / sum;
            }
            value += ostate.getProbabilityOverJointHistory(joint_history) * this->sawtooth(belief, tau);
        }
        return value;
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_solving_dpomdp>
    std::enable_if_t<is_solving_dpomdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::sawtooth(const TBelief &bstate, const number &tau)
    {
        assert(this->pomdp_vf_->getInitFunction() != nullptr);

        double v_mdp = this->pomdp_vf_->getInitFunction()->operator()(bstate, tau);
        double min_ext = 0;
        
        for (const TBelief &belief : this->pomdp_vf_->getSupport(tau))
        {
            double v_kappa = this->pomdp_vf_->operator()(belief, tau);
            double v_mdp_kappa = this->pomdp_vf_->getInitFunction()->operator()(belief, tau);
            double phi = std::numeric_limits<double>::max();
            for (auto &x : belief)
            {
                double v_int = (bstate.at(x.first) / x.second);
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }
            double min_int = phi * (v_kappa - v_mdp_kappa);
            if (min_int < min_ext)
            {
                min_ext = min_int;
            }
        }
        return v_mdp + min_ext;
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_solving_dpomdp>
    std::enable_if_t<!is_solving_dpomdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &, const number &)
    {
        throw sdm::exception::Exception("The initializer used is not available for this formalism !");
    }

    template <typename TBelief, typename TOccupancyState>
    double Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        return this->operator()<>(ostate, tau);
    }

    template <typename TBelief, typename TOccupancyState>
    double Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const Pair<TBelief, number> &belief_AND_action, const number &tau)
    {
        auto belief = belief_AND_action.first;
        auto action = belief_AND_action.second;

        return this->pomdp_vf_->getQValueAt(belief, action, tau);
    }

    template <typename TBelief, typename TOccupancyState>
    bool Belief2OccupancyValueFunction<TBelief, TOccupancyState>::isPomdpAvailable()
    {
        return true;
    }

    template <typename TBelief, typename TOccupancyState>
    bool Belief2OccupancyValueFunction<TBelief, TOccupancyState>::isMdpAvailable()
    {
        return false;
    }

    // ****** Belief Graph  ****** A faire ********* // 
    template <typename TState, typename TAction, typename TObservation>
    Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf) : pomdp_vf_(vf)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TObservation>
    double Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &, const number &)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TObservation>
    double Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const Pair<TState,number> &, const number &)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TObservation>
    bool Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isPomdpAvailable()
    {
        return true;
    }

    template <typename TState, typename TAction, typename TObservation>
    bool Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isMdpAvailable()
    {
        return false;
    }
} // namespace sdm
