/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/parser.hpp>

#include <boost/spirit/home/x3/support/ast/position_tagged.hpp>
#include <boost/spirit/home/x3/support/utility/error_reporting.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/home/x3/support/ast/position_tagged.hpp>
#include <boost/spirit/home/x3/support/utility/error_reporting.hpp>
#include <boost/spirit/home/x3/support/utility/annotate_on_success.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>

#include <map>

namespace sdm
{
  namespace parser
  {
    namespace x3 = boost::spirit::x3;

    ////////////////////////////////////////////////////////////////////////////
    //  Our error handler
    ////////////////////////////////////////////////////////////////////////////
    // X3 error handler utility
    template <typename Iterator>
    using error_handler = x3::error_handler<Iterator>;

    // tag used to get our error handler from the context
    using error_handler_tag = x3::error_handler_tag;

    struct error_handler_base
    {
      error_handler_base();

      template <typename Iterator, typename Exception, typename Context>
      x3::error_handler_result on_error(Iterator &, Iterator const &, Exception const &, Context const &);

      std::map<std::string, std::string> id_map;
    };

    ////////////////////////////////////////////////////////////////////////////
    // Implementation
    ////////////////////////////////////////////////////////////////////////////
    inline error_handler_base::error_handler_base()
    {
      id_map["dpomdp_t"] = "sdm model";
      id_map["matrix_t"] = "matrix type";
      id_map["identifier_t"] = "identifier name";
      id_map["identifiers_t"] = "identifier names";
      id_map["reward_t"] = "reward model";
      id_map["reward_entry_t"] = "reward entry";
      id_map["reward_entry_1_t"] = "reward entry of type 1";
      id_map["reward_entry_2_t"] = "reward entry of type 2";
      id_map["transition_t"] = "transition model";
      id_map["transition_entry_t"] = "transition entry";
      id_map["transition_entry_1_t"] = "transition entry of type 1";
      id_map["transition_entry_2_t"] = "transition entry of type 2";
      id_map["transition_entry_3_t"] = "transition entry of type 2";
      id_map["observation_t"] = "observation model";
      id_map["observation_entry_t"] = "observation entry";
      id_map["observation_entry_1_t"] = "observation entry of type 1";
      id_map["observation_entry_2_t"] = "observation entry of type 2";
      id_map["observation_entry_3_t"] = "observation entry of type 2";
    }

    template <typename Iterator, typename Exception, typename Context>
    inline x3::error_handler_result error_handler_base::on_error(Iterator &, Iterator const &, Exception const &x, Context const &)
    {
      std::string which = x.which();
      auto iter = id_map.find(which);
      if (iter != id_map.end())
        which = iter->second;

      std::string message = "Error! Expecting: " + which + " here:";
      // auto& error_handler = x3::get<error_handler_tag>(context)->get();
      // error_handler(x.where(), message);
      return x3::error_handler_result::fail;
    }

  } // namespace parser
} // namespace sdm
