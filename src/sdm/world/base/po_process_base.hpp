/**
 * @file po_process_base.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the partially observable process base class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/world/base/stochastic_process_base.hpp>

namespace sdm
{
    /**
     * @brief This class is the base class for partially observable processes. It contains base spaces required in all partially observable processes.
     * 
     * @tparam TStateSpace the state space
     * @tparam TObsSpace  the observation space
     * @tparam TDistrib the initial distribution
     */
    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    class PartiallyObservableProcessBase : public virtual StochasticProcessBase<TStateSpace, TDistrib>
    {
    public:
        using observation_space_type = TObsSpace;
        using observation_type = typename TObsSpace::value_type;

        PartiallyObservableProcessBase();
        PartiallyObservableProcessBase(std::shared_ptr<TStateSpace>, std::shared_ptr<TObsSpace>);
        PartiallyObservableProcessBase(std::shared_ptr<TStateSpace>, std::shared_ptr<TObsSpace>, TDistrib);

        /**
         * \brief Getter for the observation space
         */
        std::shared_ptr<TObsSpace> getObsSpace() const;

        /**
         * @brief Set the observation space
         */
        void setObsSpace(std::shared_ptr<TObsSpace>);


    protected:
        /** @brief The obervation space */
        std::shared_ptr<TObsSpace> obs_space_;
    };
} // namespace sdm
#include <sdm/world/base/po_process_base.tpp>