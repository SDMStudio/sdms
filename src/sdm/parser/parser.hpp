/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
  ==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/networked_distributed_pomdp.hpp>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <boost/spirit/home/x3.hpp>

namespace sdm
{
  namespace x3 = boost::spirit::x3;

  /**
   * @brief Namespace grouping all functions for parsing files.
   */
  namespace parser
  {
    struct dpomdp_class;

    typedef x3::rule<dpomdp_class, ast::dpomdp_t> dpomdp_type;

    BOOST_SPIRIT_DECLARE(dpomdp_type)

    std::shared_ptr<sdm::DecPOMDP> parse_string(std::string);

    std::shared_ptr<sdm::DecPOMDP> parse_file(char const *);
    
    std::shared_ptr<sdm::DecPOMDP> parse_file(std::string);

    std::shared_ptr<sdm::TwoPlayersBayesianGame> parse_file_bayesian(std::string);

  } // namespace parser
} // namespace sdm
