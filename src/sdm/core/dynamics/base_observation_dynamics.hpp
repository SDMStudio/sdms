
#pragma once
#include <sdm/utils/struct/recursive_map.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TObservation>
    class BaseObsDynamics {
        

        protected:
            //! \brief transition and observation matrices
            RecursiveMap<TAction, TState, TObservation, double> o_model; // 

            //! \brief dynamics model of the probabilities of state-observation pairs given state-action pairs.
            RecursiveMap<TState, TAction, TObservation, TState, double> dynamics; // 

        public:

            BaseObsDynamics();
            // BaseObsDynamics(BaseObsDynamics &copy);

            double getObservationProbability(TState, TAction, TObservation, TState) const;
            void setObservationProbability(TAction, TObservation, TState, double);


            double getDynamics(TState, TAction, TObservation, TState) const;
            void setDynamics(TState, TAction, TObservation, TState, double);
    };
}
#include <sdm/core/dynamics/base_observation_dynamics.tpp>
