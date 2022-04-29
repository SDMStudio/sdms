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
    struct mdp_class;
    struct mmdp_class;
    struct pomdp_class;
    struct dpomdp_class;
    struct posg_class;

    typedef x3::rule<mdp_class, ast::mdp_t> mdp_type;
    typedef x3::rule<mmdp_class, ast::mmdp_t> mmdp_type;
    typedef x3::rule<pomdp_class, ast::pomdp_t> pomdp_type;
    typedef x3::rule<dpomdp_class, ast::dpomdp_t> dpomdp_type;
    typedef x3::rule<posg_class, ast::posg_t> posg_type;

    BOOST_SPIRIT_DECLARE(mdp_type)
    BOOST_SPIRIT_DECLARE(mmdp_type)
    BOOST_SPIRIT_DECLARE(pomdp_type)
    BOOST_SPIRIT_DECLARE(dpomdp_type)
    BOOST_SPIRIT_DECLARE(posg_type)

    std::shared_ptr<MDP> parseMDP(std::string filename);
    std::shared_ptr<MMDP> parseMMDP(std::string filename);
    std::shared_ptr<POMDP> parsePOMDP(std::string filename);
    std::shared_ptr<MPOMDP> parseMPOMDP(std::string filename);
    std::shared_ptr<POSG> parsePOSG(std::string filename);

    std::shared_ptr<sdm::MPOMDP> parse_file(std::string, Config = {});

    std::shared_ptr<sdm::BayesianGameInterface> parse_file_bayesian(std::string);
    std::shared_ptr<sdm::BayesianGameInterface> parse_file_normal_form(std::string filename);

    template <typename TAst, typename TGrammar>
    TAst buildAST(std::string filename, TGrammar grammar)
    {
      TAst ast; /* Our tree */

      std::ifstream in(filename, std::ios_base::in);

      if (!in)
      {
        throw sdm::exception::FileNotFoundException(std::string(filename));
      }

      std::string storage;         /* We will read the contents here. */
      in.unsetf(std::ios::skipws); /* No white space skipping! */
      std::copy(
          std::istream_iterator<char>(in),
          std::istream_iterator<char>(),
          std::back_inserter(storage));

      /*  Defines spaces and comments */
      using boost::spirit::x3::eol;
      using boost::spirit::x3::lexeme;
      using boost::spirit::x3::ascii::char_;
      using boost::spirit::x3::ascii::space;
      auto const space_comment = space | lexeme['#' >> *(char_ - eol) >> eol];

      /*  Parse phrase (result in ast struct) */
      std::string::iterator begin = storage.begin();
      std::string::iterator iter = begin;
      std::string::iterator end = storage.end();
      bool r = phrase_parse(iter, end, grammar, space_comment, ast);
      std::string context(iter, end);

      if (r && iter == end)
      {
        return ast;
      }
      else
      {
        std::string::iterator deb_line = iter, end_line = iter;
        while (deb_line != begin && *(deb_line - 1) != '\n')
          deb_line = deb_line - 1;
        while (end_line != end && *end_line != '\n')
          end_line = end_line + 1;

        std::string context(deb_line, end_line);
        throw sdm::exception::ParsingException(context);
      }
    }

  } // namespace parser
} // namespace sdm
