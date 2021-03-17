/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <boost/spirit/home/x3/support/ast/position_tagged.hpp>
#include <boost/spirit/home/x3/support/ast/variant.hpp>

#include <vector>
#include <string>

namespace sdm{
  /**
   * @brief Namespace that is used by the parser. 
   * 
   */
  namespace ast{
    namespace x3 = boost::spirit::x3;
    ///////////////////////////////////////////////////////////////////////////
    //  Our dpomdp struct
    ///////////////////////////////////////////////////////////////////////////
    struct real_vector_t : x3::variant<std::string, std::vector<float>>{
      using base_type::base_type;
      using base_type::operator=;
    };

    struct value_t : x3::variant<unsigned short, std::vector<std::string>>{
      using base_type::base_type;
      using base_type::operator=;
    };

    struct values_t : x3::variant<std::vector<unsigned short>, std::vector<std::vector<std::string>>>{
      using base_type::base_type;
      using base_type::operator=;
    };

    struct identifier_t : x3::variant<std::string, unsigned short>{
      using base_type::base_type;
      using base_type::operator=;
    };

    typedef std::vector<identifier_t> identifiers_t;

    struct matrix_t : x3::variant<std::string, std::vector<std::vector<float>>>{
      using base_type::base_type;
      using base_type::operator=;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TRANSITION PROBABILITY
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    struct transition_entry_1_t{
      identifiers_t jaction;
      identifier_t current_state;
      identifier_t next_state;
      float probability;
    };

    struct transition_entry_2_t{
      identifiers_t jaction;
      identifier_t current_state;
      real_vector_t probabilities;
    };

    struct transition_entry_3_t{
      identifiers_t jaction;
      matrix_t transitions;
    };

    struct transition_entry_t : x3::variant<transition_entry_1_t, transition_entry_2_t, transition_entry_3_t>{
      using base_type::base_type;
      using base_type::operator=;
    };

    typedef std::vector<transition_entry_t> transition_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // OBSERVATION PROBABILITY
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    struct observation_entry_1_t{
      identifiers_t jaction;
      identifier_t next_state;
      identifiers_t next_observation;
      float probability;
    };

    struct observation_entry_2_t{
      identifiers_t jaction;
      identifier_t next_state;
      real_vector_t probabilities;
    };

    struct observation_entry_3_t{
      identifiers_t jaction;
      matrix_t probabilities;
    };

    struct observation_entry_t : x3::variant<observation_entry_1_t, observation_entry_2_t, observation_entry_3_t>{
      using base_type::base_type;
      using base_type::operator=;
    };

    typedef std::vector<observation_entry_t> observation_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // REWARD
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    struct reward_entry_1_t{
      identifiers_t jaction;
      identifier_t state;
      float reward;
    };

    struct reward_entry_2_t{
      identifiers_t jaction;
      real_vector_t rewards;
    };

    struct reward_entry_t : x3::variant<reward_entry_1_t, reward_entry_2_t>{
      using base_type::base_type;
      using base_type::operator=;
    };

    typedef std::vector<reward_entry_t> reward_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MULTI-REWARD
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //  struct multi_reward_entry_1_t{
    //   identifiers_t jaction;
    //   identifier_t state;
    //   identifier_t agent;
    //   float reward;
    // };

    // struct multi_reward_entry_2_t{
    //   identifiers_t jaction;
    //   identifier_t state;
    //   real_vector_t rewards;
    // };

    // struct multi_reward_entry_t : x3::variant<multi_reward_entry_1_t, multi_reward_entry_2_t>{
    //   using base_type::base_type;
    //   using base_type::operator=;
    // };

    // typedef std::vector<multi_reward_entry_t> multi_reward_t;

    struct dpomdp_t : x3::position_tagged{
      value_t  agent_param;
      float discount_param;
      std::string value_param;
      value_t state_param;
      real_vector_t start_param;
      values_t action_param;
      values_t observation_param;
      transition_t transition_spec;
      observation_t observation_spec;
      reward_t reward_spec;
    };
  }
}
