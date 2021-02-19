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
        using observation_type = typename TObsSpace::value_type;

        PartiallyObservableProcess();
        PartiallyObservableProcess(TStateSpace, TObsSpace);
        PartiallyObservableProcess(TStateSpace, TObsSpace, TDistrib);

        /**
         * \brief Getter for the observation spaces
         */
        TObsSpace getObsSpace() const;
        
        observation_type getObservation() const;
        
    };

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    using POProcess = PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>;

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    using PartObsProcess = PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>;
} // namespace sdm
