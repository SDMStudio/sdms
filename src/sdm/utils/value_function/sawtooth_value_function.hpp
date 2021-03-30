#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/value_function/value_function.hpp>

template<typename TState,typename TOccupancyState>
class SawtoothValueFunction : public BinaryFunction<TOccupancyState, number, double>
{
protected :
    std::shared_ptr<ValueFunction<TOccupancyState,double>> mdp_vf_;
    std::shared_ptr<ValueFunction<TOccupancyState,double>> vf_;

public:
    SawtoothValueFunction(std::shared_ptr<ValueFunction<TOccupancyState,double>> mdp_vf,std::shared_ptr<ValueFunction<TOccupancyState,double>> vf);

    template <bool is_mdp = std::is_same<TState, TOccupancyState>::value>
    std::enable_if_t<is_mdp, double>
    operator()(const TOccupancyState &ostate, const number &tau);

    template <bool is_mdp = std::is_same<TState, TOccupancyState>::value>
    std::enable_if_t<!is_mdp, double>
    operator()(const TOccupancyState &ostate, const number &tau);
    
    double operator()(const TOccupancyState &ostate, const number &tau,const TOccupancyState::state_type &newbstate);
    
    // void operator()(const TOccupancyState &bstate, cost number &tau,const TOccupancyState::state_type &newbstate)
    // {
    //     number resultat = this->mdp_vf_->getValeuAt(newbstate,tau);

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
    // }

    //Pas confiant.
    //Si je comprend bien il y a 2 Value function ? 
    // Le belief que Jilles nous a dit me paraît très différent de celui définie de base
    // Du coup faudrait aussi créer une fonction pour transformer nos occupancy states en belief states
};
