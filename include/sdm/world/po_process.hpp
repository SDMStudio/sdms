/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/world/stochastic_process.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

//!
//! \file     po_process.hpp
//! \author   David Albert
//! \brief    Partially Observable processes class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate partially observable stochastic processes.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  PartiallyObservableProcess
    //! \brief Partially observable process
    class PartiallyObservableProcess : public virtual StochasticProcess
    {

    protected:
        //! \brief The state space
        MultiDiscreteSpace<number> obs_spaces_;

    public:
        PartiallyObservableProcess();

        PartiallyObservableProcess(const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &);
        PartiallyObservableProcess(const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &, const Vector &);

        /**
         * \brief Getter for the observation spaces
         */
        const MultiDiscreteSpace<number> &getObsSpace() const;

        /**
         * \brief Get the number of joint observations
         */
        number getNumJObservations() const;

        /**
         * \brief Get the number of observations for a specific agent
         */
        number getNumObservations(number) const;

        /**
         * \brief Get the number of observation for every agents
         */
        std::vector<number> getNumObservations() const;
    };

    typedef PartiallyObservableProcess POProcess;
    typedef PartiallyObservableProcess PartObsProcess;
} // namespace sdm
