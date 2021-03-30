#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>

namespace sdm
{
    template <typename TBelief, typename TOccupancyState>
    class Belief2OccupancyValueFunction : public BinaryFunction<TOccupancyState, number, double>
    {
    protected:
        std::shared_ptr<BinaryFunction<TBelief, number, double>> pomdp_vf_;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TBelief, number, double>> pomdp_vf) : pomdp_vf_(pomdp_vf)
        {
        }

        double sawtooth(const TBelief &bstate, const number &tau)
        {
            // sawtooth

            number resultat = this->pomdp_vf_->getValueAt(bstate, tau);

            number min_ext = 0;
            for (auto &p_b_v : this->pomdp_vf_->getValueFunctionAt(tau))
            {
                number min = 0;
                auto belief = p_b_v.first;
                auto value = p_b_v.second;
                for (auto &x : belief)
                {
                    double v_int = (bstate[x.first] / x.second) * (value - this->pomdp_vf_->getValueAt(belief));
                    min = std::min(min, v_int);
                }
                min_ext = std::min(min_ext, min);
            }

            // number value_bstate = bstate.second;
            // number vf_bstate = this->vf_->getValueAt(bstate, tau);

            // number min;

            // for (const auto &belief : ostate)
            // {
            //     auto state = belief.first.getState();
            //     auto proba = belief.second;
            //     if (proba > 0) // Si proba est supérieur à 0, alors
            //     {
            //         ratio = value_bstate / proba * (vf_bstate - this->mdp_vf_->getValueAt(belief, tau))

            //                                            if (ratio < min)
            //         {
            //             min = ratio;
            //         }
            //     }
            // }
            return resultat + min_ext;
        }

        double operator()(const TOccupancyState &ostate, const number &tau)
        {
            double value = 0;
            std::map<TOccupancyState::jhistory_type, TBelief> belief_map; // Pas possible de faire decltype ici car o n'est pas encore déclaré TOccupancyState::jhistory_type plutot
            std::map<TOccupancyState::jhistory_type, double> o_proba;

            for (const auto &ost : ostate)
            {
                auto x = TOccupancyState::getState(ost.first);
                auto o = TOccupancyState::getHistory(ost.first);
                auto p = ost.second;
                o_proba[o] += p;       // Idem
                belief_map[o][x] += p; // Pas possible direct. Il faut d'abord checker qu'une valeur existe avec belief_map.find et = ou += ensuite
            }

            for (const auto &p_o_p : o_proba)
            {
                TBelief belief = belief_map[p_o_p.first] : double sum = belief.norm_1();
                for (const auto &b_s : belief)
                {
                    belief[b_s.first] = belief[b_s.first] / sum;
                }
                value += p_o_p.second * this->sawtooth(belief, tau);
            }
            return value;
        }

        // template <bool is_mdp = std::is_same<TState, TOccupancyState>::value>
        // std::enable_if_t<is_mdp, double>
        // operator()(const TOccupancyState &ostate, const number &tau);

        // template <bool is_mdp = std::is_same<TState, TOccupancyState>::value>
        // std::enable_if_t<!is_mdp, double>
        // operator()(const TOccupancyState &ostate, const number &tau);

        // double operator()(const TOccupancyState &ostate, const number &tau);

        //Pas confiant.
        //Si je comprend bien il y a 2 Value function ?
        // Le belief que Jilles nous a dit me paraît très différent de celui définie de base
        // Du coup faudrait aussi créer une fonction pour transformer nos occupancy states en belief states
    };

} // namespace sdm

#include <sdm/utils/value_function/belief_2_occupancy_vf.tpp>
