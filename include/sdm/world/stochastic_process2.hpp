// /*=============================================================================
//   Copyright (c) 2020 David Albert
// ==============================================================================*/
// #pragma once

// #include <random>
// #include <vector>
// #include <sdm/types.hpp>
// #include <sdm/exception.hpp>
// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/utils/linear_algebra/vector.hpp>

// //!
// //! \file     stochastic_process.hpp
// //! \author   David Albert
// //! \brief    Generic Stochastic process class
// //! \version  1.0
// //! \date     24 novembre 2020
// //!
// //! This abstract class provide useful members for all stochastic processes.

// //! \namespace  sdm
// //! \brief Namespace grouping all tools required for sequential decision making.
// namespace sdm
// {
//     //! \class  StochasticProcess
//     template <typename TStateSpace, typename TDistrib>
//     class StochasticProcess2
//     {
//     private:
//         /**
//          * @brief The internal state
//          */
//         value_type internal_state_ = 0;

//     protected:
//         /**
//          * @brief The state space
//          */
//         TStateSpace state_space_;

//         /**
//          * @brief The initial state distribution
//          */
//         TDistrib start_distrib_;

//     public:
//         using value_type = TStateSpace::value_type;

//         StochasticProcess();
//         StochasticProcess(const TStateSpace &);
//         StochasticProcess(const TStateSpace &, const TDistrib &);

//         value_type getInternalState() const;
//         void setInternalState(value_type new_i_state);

//         /**
//          * @brief Init the process and return the initial internal state. This initial state is sampled from the "start" distribution.
//          * 
//          * @return value_type the sampled initial state
//          */
//         value_type init();

//         const TStartDistrib &getStartDistrib() const;
//         const TStateSpace &getStateSpace() const;

//         void setStartDistrib(const TStartDistrib &);
//         void setStateSpace() const;

//         virtual TDistrib getProbaNextState(value_type)
//         {
//             throw sdm::exception::NotImplementedException();
//         };

//         /**
//          * @brief Execute a step and return the next sampled state
//          * 
//          * @return value_type 
//          */
//         virtual value_type nextState()
//         {
//             throw sdm::exception::NotImplementedException();
//         };
//     };
// } // namespace sdm
