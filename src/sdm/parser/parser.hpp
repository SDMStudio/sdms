/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
  ==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <boost/spirit/home/x3.hpp>

namespace sdm
{
  namespace x3 = boost::spirit::x3;

  ///////////////////////////////////////////////////////////////////////////////
  // dpomdp public interface
  ///////////////////////////////////////////////////////////////////////////////
  namespace parser
  {
    struct dpomdp_class;

    typedef x3::rule<dpomdp_class, ast::dpomdp_t> dpomdp_type;

    BOOST_SPIRIT_DECLARE(dpomdp_type)

    std::shared_ptr<sdm::DiscreteDecPOMDP> parse_string(std::string);

    std::shared_ptr<sdm::DiscreteDecPOMDP> parse_file(char const *);
    
    std::shared_ptr<sdm::DiscreteDecPOMDP> parse_file(std::string);

  } // namespace parser
} // namespace sdm
