#include <sdm/utils/value_function/belief_2_occupancy_vf.hpp>

namespace sdm
{

    template <typename TBelief, typename TOccupancyState>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TBelief, number>> pomdp_vf) : pomdp_vf_(pomdp_vf)
    {
        std::cout << *pomdp_vf << std::endl;
        // std::cout << *pomdp_vf->getInitFunction() << std::endl;
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_solving_dpomdp>
    std::enable_if_t<is_solving_dpomdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        double value = 0;

        std::map<typename TOccupancyState::jhistory_type, double> o_proba;
        std::map<typename TOccupancyState::jhistory_type, TBelief> belief_map; // Pas possible de faire decltype ici car o n'est pas encore déclaré TOccupancyState::jhistory_type plutot

        for (const auto &ost : ostate)
        {
            auto x = TOccupancyState::getState(ost.first);
            auto o = TOccupancyState::getHistory(ost.first);
            auto p = ost.second;
            o_proba[o] += p;
            belief_map[o][x] += p;
        }

        // $sum_{o_{\tau}} p(o_{\tau} \mid s_{\tau} v_{\tau}^{pomdp}\left( x_{\tau} \mid o_{\tau} \right))$
        for (const auto &p_o_p : o_proba)
        {
            TBelief belief = belief_map[p_o_p.first];
            double sum = belief.norm_1();
            for (const auto &b_s : belief)
            {
                belief[b_s.first] = belief[b_s.first] / sum;
            }
            // std::cout << "belief : " << belief << " --- p(o | ostate) =" << p_o_p.second << std::endl;
            value += p_o_p.second * this->sawtooth(belief, tau);
        }
        return value;
    }

    // template <typename TBelief, typename TOccupancyState>
    // template <bool is_solving_mdp>
    // std::enable_if_t<is_solving_mdp, double>
    // Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    // {
    //     // return this->pomdp_vf_->operator()(ostate, tau);
    //     return 0.0;
    // }

    // template <typename TBelief, typename TOccupancyState>
    // template <bool is_solving_pomdp>
    // std::enable_if_t<is_solving_pomdp, double>
    // Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    // {
    //     // return this->pomdp_vf_->operator()(ostate, tau);
    //     return 0.0;
    // }

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
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        // return this->pomdp_vf_->operator()(ostate, tau);
        return 0.0;
    }

    template <typename TBelief, typename TOccupancyState>
    double Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        return this->operator()<>(ostate, tau);
    }

} // namespace sdm
