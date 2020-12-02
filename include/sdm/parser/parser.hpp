/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
  ==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/world/decpomdp.hpp>
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
    // struct ndpomdp_class;

    typedef x3::rule<dpomdp_class, ast::dpomdp_t> dpomdp_type;
    // typedef x3::rule<ndpomdp_class, ast::ndpomdp_t> ndpomdp_type;

    BOOST_SPIRIT_DECLARE(dpomdp_type)
    // BOOST_SPIRIT_DECLARE(ndpomdp_type)

    sdm::DecPOMDP parse_string(std::string);

    sdm::DecPOMDP parse_file(char const *);
    
    sdm::DecPOMDP parse_file(std::string);

  } // namespace parser

  // parser::dpomdp_type const& dpomdp();
  // parser::ndpomdp_type const& ndpomdp();
} // namespace sdm
