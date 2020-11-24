/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
  ==============================================================================*/
#pragma once

#include <sdm/parser/error_handler.hpp>

#include <boost/spirit/home/x3.hpp>

namespace sdm{
  namespace parser{
    // our iterator type
    typedef std::string::const_iterator iterator_type;

    // the phrase parse context
    typedef x3::phrase_parse_context<x3::ascii::space_type>::type phrase_context_type;

    // our error handler
   typedef error_handler<iterator_type> error_handler_type;

    // combined error handler and phrase parse context
    typedef x3::with_context<error_handler_tag, std::reference_wrapper<error_handler_type> const, phrase_context_type>::type context_type;
  }
}
