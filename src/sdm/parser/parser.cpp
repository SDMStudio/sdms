/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/exception.hpp>
#include <sdm/parser/ast.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/parser/config.hpp>
#include <sdm/parser/printer.hpp>
#include <sdm/parser/encoder.hpp>
#include <sdm/parser/parser_def.hpp>
#include <sdm/parser/ast_adapted.hpp>

#include <boost/spirit/home/x3.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/config/warning_disable.hpp>

#include <regex>
#include <string>
#include <fstream>
#include <iostream>

namespace sdm
{
  namespace parser
  {
    BOOST_SPIRIT_INSTANTIATE(dpomdp_type, iterator_type, context_type)

    std::shared_ptr<sdm::DecPOMDP> parse_string(std::string storage)
    {
      using sdm::parser::dpomdp_t; // Our grammar
      sdm::ast::dpomdp_t ast;      // Our tree

      // Defines spaces and comments
      using boost::spirit::x3::eol;
      using boost::spirit::x3::lexeme;
      using boost::spirit::x3::ascii::char_;
      using boost::spirit::x3::ascii::space;
      auto const space_comment = space | lexeme['#' >> *(char_ - eol) >> eol];

      // Parse phrase (result in ast struct)
      std::string::iterator iter = storage.begin();
      std::string::iterator end = storage.end();
      bool r = phrase_parse(iter, end, dpomdp_t, space_comment, ast);
      std::string context(iter, end);
      if (r && iter == end)
      {
        // Convert ast to DPOMDP class
        sdm::ast::dpomdp_encoder encoder;
        return encoder(ast);
      }
      else
      {
        std::string::iterator some = iter + 30;
        std::string context(iter, (some > end) ? end : some);
        throw sdm::exception::ParsingException(context);
      }
    }

    std::shared_ptr<sdm::DecPOMDP> parse_file(char const *filename)
    {
      std::ifstream in(filename, std::ios_base::in);

      if (!in)
      {
        throw sdm::exception::FileNotFoundException(std::string(filename));
      }

      std::string storage;         // We will read the contents here.
      in.unsetf(std::ios::skipws); // No white space skipping!
      std::copy(
          std::istream_iterator<char>(in),
          std::istream_iterator<char>(),
          std::back_inserter(storage));

      std::shared_ptr<sdm::DecPOMDP> parsed_model = sdm::parser::parse_string(storage);
      // parsed_model->setFileName(filename);
      return parsed_model;
    }

    std::shared_ptr<sdm::DecPOMDP> parse_file(std::string filename)
    {
      if (regex_match(filename, std::regex(".*\\.ndpomdp$")) || regex_match(filename, std::regex(".*\\.NDPOMDP$")))
      {
        return std::make_shared<NetworkedDistributedPOMDP>(filename);
      }
      else
      {
        return parse_file(filename.c_str());
      }
    }

  } // namespace parser

  // parser::dpomdp_type const& dpomdp(){
  //   return sdm::parser::dpomdp_t;
  // }

} // namespace sdm
