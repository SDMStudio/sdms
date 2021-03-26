#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/value_function/value_function.hpp>

template<typename TState,typename TBelief>
class SawtoothValueFunction : public BinaryFunction<TBelief, number, double>
{
protected :
    std::shared_ptr<ValueFunction<TBelief,double>> mdp_vf_;

public:
    SawtoothValueFunction(std::shared_ptr<ValueFunction<TBelief,double>> vf) : mdp_vf_(vf)
    {}

    void operator()(const TBelief &bstate,cost number &tau)
    {
        number resultat = this->mdp_vf_->getValeuAt(belief,tau);
        
        number min;
        number argmin; 

        for()
 
    }

    number resultat(const TBelief &bstate,cost number &tau,const TObservation &history)
    {
        number min;
        number argmin; 

        number ratio;
        
        for (const auto &belief : bstate)
        {
            auto state = belief.first.getState();
            auto proba = belief.second;
            if (proba>0) // Si proba est supérieur à 0, alors 
            {
                //ratio = /proba
                //(TBelief(state,history)-this->mdp_vf_->getValeuAt(belief,tau))


            }
        }
    }
};
