/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>

#include <boost/fusion/include/adapt_struct.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::observation_entry_1_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::identifier_t, next_state)
  (sdm::ast::identifiers_t, next_observation)
  (float, probability)
)

// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::observation_entry_2_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::identifier_t,  next_state)
  (sdm::ast::real_vector_t,  probabilities)
)

// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::observation_entry_3_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::matrix_t,  probabilities)
)


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::transition_entry_1_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::identifier_t, current_state)
  (sdm::ast::identifier_t, next_state)
  (float, probability)
)

// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::transition_entry_2_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::identifier_t, current_state)
  (sdm::ast::real_vector_t,  probabilities)
)

// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::transition_entry_3_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::matrix_t,  transitions)
)


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::reward_entry_1_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::identifier_t, state)
  (float, reward)
)

// We need to tell fusion about our reward_entry_1_t struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::reward_entry_2_t,
  (sdm::ast::identifiers_t, jaction)
  (sdm::ast::real_vector_t,  rewards)
)


// We need to tell fusion about our dpomdp struct
// to make it a first-class fusion citizen. This has to
// be in global scope.
BOOST_FUSION_ADAPT_STRUCT(sdm::ast::dpomdp_t,
  (sdm::ast::value_t, agent_param)
  (float, discount_param)
  (std::string, value_param)
  (sdm::ast::value_t, state_param)
  (sdm::ast::real_vector_t, start_param)
  (sdm::ast::values_t, action_param)
  (sdm::ast::values_t, observation_param)
  (sdm::ast::transition_t, transition_spec)
  (sdm::ast::observation_t, observation_spec)
  (sdm::ast::reward_t, reward_spec)
)
