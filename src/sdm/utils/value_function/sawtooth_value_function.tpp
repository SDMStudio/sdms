#include <sdm/utils/value_function/sawtooth_value_function.hpp>

namespace sdm
{

    template <typename TState, typename TOccupancyState>
    SawtoothValueFunction<TState, TOccupancyState>::SawtoothValueFunction(std::shared_ptr<ValueFunction<TOccupancyState,double>> mdp_vf,std::shared_ptr<ValueFunction<TOccupancyState,double>> vf) : mdp_vf_(mdp_vf),vf_(vf)
    {
    }

    template <typename TState, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<is_mdp, double>
    SawtoothValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        return this->mdp_vf_->operator()(ostate, tau);
    }

    template <typename TState, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<!is_mdp, double>
    SawtoothValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau,const typename TOccupancyState::state_type &newbstate)
    {
        double value = this->mdp_vf_->getValeuAt(newbstate,tau)


        double min = std::numeric_limits<double>::max();
        for (auto &ost : ostate)
        {
            auto state = TOccupancyState::getState(ost.first);
            auto proba = ost.second;

            if (proba>0) // Si proba est supérieur à 0, alors 
            {
                min = std::min(value_newbstate/proba * (this->vf_->getValeuAt(ost,tau)-this->mdp_vf_->getValeuAt(ost,tau)),min);
            }

            //value += proba * this->mdp_vf_->operator()(state, tau);
        }
        return value + min;

        //number resultat = this->mdp_vf_->getValeuAt(newbstate,tau);

    //     number value_newbstate = newbstate.second;
    //     number vf_newbstate = this->vf_->getValeuAt(newbstate,tau);    
    //     number min;
    //     for (const auto &belief : bstate)
    //     {
    //         auto state = belief.first.getState();
    //         auto proba = belief.second;
    //         if (proba>0) // Si proba est supérieur à 0, alors 
    //         {
    //             ratio = value_newbstate/proba * (vf_newbstate-this->mdp_vf_->getValeuAt(belief,tau))
    //             if(ratio<min)
    //             {
    //                 min = ratio; 
    //             }
    //         }
    //     }
    //     return resultat + min;
    }

    template <typename TState, typename TOccupancyState>
    double SawtoothValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau,const typename TOccupancyState::state_type &newbstate)
    {
        return this->operator()<>(ostate, tau,newbstate);
    }

} // namespace sdm
