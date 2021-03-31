#include <sdm/utils/value_function/belief_2_occupancy_vf.hpp>

namespace sdm
{

    template <typename TBelief, typename TOccupancyState>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::Belief2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TBelief, number, double>> pomdp_vf) : pomdp_vf_(pomdp_vf)
    {
    }

    
    template <typename TBelief, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<is_mdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        double value =0;

        std::map<typename TOccupancyState::jhistory_type, TBelief> belief_map; // Pas possible de faire decltype ici car o n'est pas encore déclaré TOccupancyState::jhistory_type plutot
        std::map<typename TOccupancyState::jhistory_type, double> o_proba;

        for (const auto &ost : ostate)
        {
            auto x = TOccupancyState::getState(ost.first);
            auto o = TOccupancyState::getHistory(ost.first);
            auto p = ost.second;
            o_proba[o] += p;       // Idem
            belief_map[o][x] += p; // Pas possible direct. Il faut d'abord checker qu'une valeur existe avec belief_map.find et = ou += ensuite

            this->getAllBelief.insert(x);
        }

        for (const auto &p_o_p : o_proba)
        {
            TBelief belief = belief_map[p_o_p.first]; 
            double sum = belief.norm_1();
            for (const auto &b_s : belief)
            {
                belief[b_s.first] = belief[b_s.first] / sum;
            }
            value += p_o_p.second * this->sawtooth(belief, tau);
            std::cout<<"\n sawtooth : "<<this->sawtooth(belief, tau)<<", pop.second : "<<p_o_p.second<<", value : "<<value;
        }
        //std::cout<<"\n osate :"<<ostate<<"\n value : "<<value;
        return value;
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<!is_mdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        std::cout<<"\n bbbbbbbbbbbbbbbbb";
        return 0.0;
    }

    template <typename TBelief, typename TOccupancyState>
    MappedVector<TBelief,double> Belief2OccupancyValueFunction<TBelief, TOccupancyState>::getMappedBelief(const number &tau)
    {
        MappedVector<TBelief,double> mapped_vector(0);

        for(const TBelief &belief : this->getAllBelief)
        {
            mapped_vector[belief] = 1;//this->pomdp_vf_->operator()(belief,tau);
        }
        return mapped_vector;        
    }

    template <typename TBelief, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<is_mdp, double>
    Belief2OccupancyValueFunction<TBelief, TOccupancyState>::sawtooth(const TBelief &bstate, const number &tau)
    {
        number resultat = this->pomdp_vf_->operator()(bstate, tau);

        number min_ext = 0;

        for(const TBelief &belief : this->getAllBelief)
        {
            auto value = this->pomdp_vf_->operator()(belief,tau);
            number min_int = 0;

            if(value>0)
            {
                for (auto &x : belief)
                {
                    std::cout<<"222222222222222222222222222222222222222222222";
                    double v_int = (bstate.at(x.first) / x.second)* (value - this->pomdp_vf_->operator()(bstate,tau));
                    if(v_int<min_int)
                    {
                        min_int = v_int;
                    }
                }
            }
            if(min_int<min_ext)
            {
                min_ext = min_int;
            }
        }
        // for (auto &p_b_v : mapped_vector_tau);
        // {
        //     number min_int = 0;
        //     std::cout<<"\n size : "<<mapped_vector_tau.size();
        //     // auto belief = p_b_v.first;
        //     // auto value = p_b_v.second;
        //     // for (auto &x : belief)
        //     // {
        //     //     double v_int = (bstate[x.first] / x.second) * (value - this->pomdp_vf_->getValueAt(belief));
        //     //     //min_int = std::min(min_int, v_int);
        //     // }
        //     //min_ext = std::min(min_ext, min_int);
        // }
        return resultat + min_ext;
    }

    template <typename TBelief, typename TOccupancyState>
    double Belief2OccupancyValueFunction<TBelief, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        std::cout<<"\n need me !!, ostate : "<<ostate;
        return this->operator()<>(ostate, tau);
    }

} // namespace sdm
