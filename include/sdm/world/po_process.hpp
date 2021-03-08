/**
 * @file po_process.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/stochastic_process.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    class PartiallyObservableProcess : public virtual StochasticProcess<TStateSpace, TDistrib>
    {

    protected:
        //! \brief The state space
        TObsSpace obs_spaces_;

    public:
        using observation_space_type = TObsSpace;
        using observation_type = typename TObsSpace::value_type;
        using state_type = typename StochasticProcess<TStateSpace, TDistrib>::state_type;

        PartiallyObservableProcess();
        PartiallyObservableProcess(TStateSpace, TObsSpace);
        PartiallyObservableProcess(TStateSpace, TObsSpace, TDistrib);

        /**
         * \brief Getter for the observation spaces
         */
        TObsSpace getObsSpace() const;

        /**
         * @brief Set the observation space
         */
        void setObsSpace(TObsSpace) const;        

        virtual TDistrib getProbaObservation(state_type)
        {
            throw sdm::exception::NotImplementedException();
        }

        /**
         * @brief Execute a step and return the next sampled state
         * 
         * @return state_type 
         */
        virtual observation_type getObservation()
        {
            return this->getProbaObservation(this->getInternalState())(sdm::common::global_urng());
        }

    };

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    using POProcess = PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>;

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    using PartObsProcess = PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>;
} // namespace sdm
#include <sdm/world/po_process.tpp>
