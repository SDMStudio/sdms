/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
  ==============================================================================*/
#pragma once

#include <sdm/macros.hpp>
#include <sdm/parser/ast.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/world/networked_distributed_pomdp.hpp>
#include <sdm/world/bayesian_game_interface.hpp>
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
    struct posg_class;

    typedef x3::rule<dpomdp_class, ast::dpomdp_t> dpomdp_type;
    typedef x3::rule<posg_class, ast::posg_t> posg_type;

    BOOST_SPIRIT_DECLARE(dpomdp_type)
    BOOST_SPIRIT_DECLARE(posg_type)

    SDMS_DECLARE_PARSER(sdm::MDP, parseMDP)
    SDMS_DECLARE_PARSER(sdm::POMDP, parsePOMDP)
    SDMS_DECLARE_PARSER(sdm::MPOMDP, parseMPOMDP)
    SDMS_DECLARE_PARSER(sdm::DecPOMDP, parseDecPOMDP)
    SDMS_DECLARE_PARSER(sdm::POSG, parsePOSG)

    std::shared_ptr<sdm::MPOMDP> parse_file(std::string, Config = {});

    std::shared_ptr<sdm::BayesianGameInterface> parse_file_bayesian(std::string);
    std::shared_ptr<sdm::BayesianGameInterface> parse_file_normal_form(std::string filename);

  } // namespace parser
} // namespace sdm
