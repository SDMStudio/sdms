/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/exception.hpp>
#include <sdm/tools.hpp>
#include <sdm/parser/ast.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/parser/config.hpp>
#include <sdm/parser/printer.hpp>
#include <sdm/parser/encoder.hpp>
#include <sdm/parser/parser_def.hpp>
#include <sdm/parser/ast_adapted.hpp>

#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/world/two_players_normal_form_game.hpp>

#include <boost/spirit/home/x3.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/config/warning_disable.hpp>

#include <regex>
#include <string>
#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>

namespace sdm
{
  namespace parser
  {

    BOOST_SPIRIT_INSTANTIATE(mdp_type, iterator_type, context_type)
    BOOST_SPIRIT_INSTANTIATE(mmdp_type, iterator_type, context_type)
    BOOST_SPIRIT_INSTANTIATE(pomdp_type, iterator_type, context_type)
    BOOST_SPIRIT_INSTANTIATE(dpomdp_type, iterator_type, context_type)
    BOOST_SPIRIT_INSTANTIATE(posg_type, iterator_type, context_type)

    std::shared_ptr<MDP> parseMDP(std::string filename)
    {
      filename = sdm::tools::getWorldPath(filename);
      sdm::ast::mdp_encoder encoder;
      if (sdm::tools::hasExtension(filename, "mdp") || sdm::tools::hasExtension(filename, "MDP"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::mdp_t ast = buildAST<sdm::ast::mdp_t>(filename, sdm::parser::mdp_t);
        return encoder(ast);
      }
      else
      {
        throw sdm::exception::Exception("Cannot parse '" + filename + "' into MMDP.");
      }
    }

    std::shared_ptr<MMDP> parseMMDP(std::string filename)
    {
      filename = sdm::tools::getWorldPath(filename);
      sdm::ast::mmdp_encoder encoder;
      if (sdm::tools::hasExtension(filename, "mdp") || sdm::tools::hasExtension(filename, "MDP"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::mdp_t ast = buildAST<sdm::ast::mdp_t>(filename, sdm::parser::mdp_t);
        return encoder(ast, false);
      }
      else if (sdm::tools::hasExtension(filename, "mmdp") || sdm::tools::hasExtension(filename, "MMDP"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::mmdp_t ast = buildAST<sdm::ast::mmdp_t>(filename, sdm::parser::mmdp_t);
        return encoder(ast);
      }
      else if (sdm::tools::hasExtension(filename, "dpomdp") || sdm::tools::hasExtension(filename, "mpomdp"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::dpomdp_t ast = buildAST<sdm::ast::dpomdp_t>(filename, sdm::parser::dpomdp_t);
        return encoder(ast);
      }
      else
      {
        throw sdm::exception::Exception("Cannot parse '" + filename + "' into MMDP.");
      }
    }

    std::shared_ptr<POMDP> parsePOMDP(std::string filename)
    {
      filename = sdm::tools::getWorldPath(filename);
      sdm::ast::pomdp_encoder encoder;
      if (sdm::tools::hasExtension(filename, "pomdp") || sdm::tools::hasExtension(filename, "POMDP"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::pomdp_t ast = buildAST<sdm::ast::pomdp_t>(filename, sdm::parser::pomdp_t);
        return encoder(ast);
      }
      else
      {
        throw sdm::exception::Exception("Cannot parse '" + filename + "' into POMDP.");
      }
    }

    std::shared_ptr<MPOMDP> parseMPOMDP(std::string filename)
    {
      filename = sdm::tools::getWorldPath(filename);
      sdm::ast::dpomdp_encoder encoder;
      if (sdm::tools::hasExtension(filename, "pomdp") || sdm::tools::hasExtension(filename, "POMDP"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::pomdp_t ast = buildAST<sdm::ast::pomdp_t>(filename, sdm::parser::pomdp_t);
        return encoder(ast);
      }
      else if (sdm::tools::hasExtension(filename, "dpomdp") || sdm::tools::hasExtension(filename, "mpomdp"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::dpomdp_t ast = buildAST<sdm::ast::dpomdp_t>(filename, sdm::parser::dpomdp_t);
        return encoder(ast);
      }
      else
      {
        throw sdm::exception::Exception("Cannot parse '" + filename + "' into MPOMDP.");
      }
    }

    std::shared_ptr<POSG> parsePOSG(std::string filename)
    {
      filename = sdm::tools::getWorldPath(filename);
      sdm::ast::posg_encoder encoder;
      if (sdm::tools::hasExtension(filename, "posg") || sdm::tools::hasExtension(filename, "POSG"))
      {
        /* Convert ast to MMDP class */
        sdm::ast::posg_t ast = buildAST<sdm::ast::posg_t>(filename, sdm::parser::posg_t);
        return encoder(ast);
      }
      else
      {
        throw sdm::exception::Exception("Cannot parse '" + filename + "' into POSG.");
      }
    }

    std::shared_ptr<sdm::MPOMDP> parse_file(std::string filename, Config config)
    {
      filename = sdm::tools::getWorldPath(filename);
      if (tools::hasExtension(filename, "ndpomdp") || tools::hasExtension(filename, "NDPOMDP"))
      {
        return std::make_shared<NetworkedDistributedPOMDP>(filename);
      }
      else if (tools::hasExtension(filename, "posg") || tools::hasExtension(filename, "POSG"))
      {
        auto posg = parsePOSG(filename.c_str());
        posg->configure(config);
        return posg;
      }
      else
      {
        auto dpomdp = parseMPOMDP(filename.c_str());
        dpomdp->configure(config);
        return dpomdp;
      }
    }

    std::shared_ptr<sdm::BayesianGameInterface> parse_file_bayesian(std::string filename)
    {

      auto split = [](const std::string chaine, char delimiteur)
      {
        std::vector<std::string> elements;
        std::stringstream ss(chaine);
        std::string sousChaine;
        while (getline(ss, sousChaine, delimiteur))
        {
          elements.push_back(sousChaine);
        }
        return elements;
      };
      std::shared_ptr<sdm::TwoPlayersBayesianGame> game;
      if (regex_match(filename, std::regex(".*\\.byg$")) || regex_match(filename, std::regex(".*\\.BYG$")))
      {
        game = std::make_shared<sdm::TwoPlayersBayesianGame>(sdm::TwoPlayersBayesianGame());
      }
      else
      {
        return nullptr; // right way ?
      }
      std::ifstream inputFile(filename);
      if (!inputFile)
      {
        throw sdm::exception::FileNotFoundException(std::string(filename));
      }
      std::string line;
      getline(inputFile, line);
      std::vector<std::string> lineElements(split(line, ' '));

      // get problem dimensions
      game->setTypeNumbers(lineElements);

      std::vector<int> typesNumbers = game->getTypesNumbers();

      getline(inputFile, line);
      lineElements = split(line, ' ');
      game->setGameDimensions(lineElements);
      std::vector<int> matrixDimensions = game->getGameDimensions();

      // get payoffMatrix
      for (int i = 0; i < typesNumbers[0] * typesNumbers[1] * matrixDimensions[0] * 2; i++)
      {
        getline(inputFile, line);
        lineElements = split(line, ' ');
        game->addPayoffLine(lineElements);
      }

      // get joint probabilities
      for (int i = 0; i < typesNumbers[0]; i++)
      {
        getline(inputFile, line);
        lineElements = split(line, ' ');
        game->addJointTypeProbabilities(lineElements);
      }

      return game;
    }

    std::shared_ptr<sdm::BayesianGameInterface> parse_file_normal_form(std::string filename)
    {

      auto split = [](const std::string chaine, char delimiteur)
      {
        std::vector<std::string> elements;
        std::stringstream ss(chaine);
        std::string sousChaine;
        while (getline(ss, sousChaine, delimiteur))
        {
          elements.push_back(sousChaine);
        }
        return elements;
      };
      std::shared_ptr<sdm::TwoPlayersNormalFormGame> game;
      if (regex_match(filename, std::regex(".*\\.nfg$")) || regex_match(filename, std::regex(".*\\.NFG$")))
      {
        game = std::make_shared<sdm::TwoPlayersNormalFormGame>(sdm::TwoPlayersNormalFormGame());
      }
      else
      {
        return nullptr; // right way ?
      }
      std::ifstream inputFile(filename);
      if (!inputFile)
      {
        throw sdm::exception::FileNotFoundException(std::string(filename));
      }
      std::string line;
      getline(inputFile, line);
      std::vector<std::string> lineElements(split(line, ' '));

      std::vector<int> typesNumbers = game->getTypesNumbers();
      game->setGameDimensions(lineElements);
      std::vector<int> matrixDimensions = game->getGameDimensions();

      // get payoffMatrix
      for (int i = 0; i < typesNumbers[0] * typesNumbers[1] * matrixDimensions[0] * 2; i++)
      {
        getline(inputFile, line);
        lineElements = split(line, ' ');
        game->addPayoffLine(lineElements);
      }

      return game;
    }

    std::shared_ptr<sdm::BayesianGameInterface> parse_file_bayesian_game(std::string filename)
    {
      if (regex_match(filename, std::regex(".*\\.nfg$")) || regex_match(filename, std::regex(".*\\.NFG$")))
      {
        return parse_file_normal_form(filename);
      }
      else if (regex_match(filename, std::regex(".*\\.byg$")) || regex_match(filename, std::regex(".*\\.BYG$")))
      {
        return parse_file_bayesian(filename);
      }
      else
      {
        throw sdm::exception::Exception("File format not supported by parse_file_bayesian");
      }
    }

  } // namespace parser

  // parser::dpomdp_type const& dpomdp(){
  //   return sdm::parser::dpomdp_t;
  // }

} // namespace sdm
