/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/parser/ast_adapted.hpp>
#include <sdm/parser/error_handler.hpp>

#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/home/x3/support/utility/annotate_on_success.hpp>

namespace sdm
{
  namespace parser
  {
    namespace x3 = boost::spirit::x3;
    namespace ascii = boost::spirit::x3::ascii;

    using x3::alnum;
    using x3::alpha;
    using x3::eol;
    using x3::float_;
    using x3::lexeme;
    using x3::lit;
    using x3::raw;
    using x3::ushort_;

    using ascii::blank;
    using ascii::char_;
    using ascii::space;

    ///////////////////////////////////////////////////////////////////////////
    // Rule IDs
    ///////////////////////////////////////////////////////////////////////////
    struct identifier_t_class;
    struct identifiers_t_class;

    struct matrix_t_class;

    struct reward_t_class;
    struct reward_entry_t_class;
    struct reward_entry_1_t_class;
    struct reward_entry_2_t_class;

    struct multi_reward_t_class;
    struct multi_reward_entry_t_class;
    struct multi_reward_entry_1_t_class;
    struct multi_reward_entry_2_t_class;

    struct transition_t_class;
    struct transition_entry_t_class;
    struct transition_entry_1_t_class;
    struct transition_entry_2_t_class;
    struct transition_entry_3_t_class;

    struct observation_t_class;
    struct observation_entry_t_class;
    struct observation_entry_1_t_class;
    struct observation_entry_2_t_class;
    struct observation_entry_3_t_class;

    ///////////////////////////////////////////////////////////////////////////
    // Rules
    ///////////////////////////////////////////////////////////////////////////
    mdp_type const mdp_t = "mdp_t";
    mmdp_type const mmdp_t = "mmdp_t";
    pomdp_type const pomdp_t = "pomdp_t";
    dpomdp_type const dpomdp_t = "dpomdp_t";
    posg_type const posg_t = "posg_t";

    x3::rule<identifier_t_class, ast::identifier_t> const identifier_t = "identifier_t";
    x3::rule<identifiers_t_class, ast::identifiers_t> const identifiers_t = "identifiers_t";

    x3::rule<matrix_t_class, ast::matrix_t> const matrix_t = "matrix_t";

    x3::rule<reward_t_class, ast::reward_t> const reward_t = "reward_t";
    x3::rule<reward_entry_t_class, ast::reward_entry_t> const reward_entry_t = "reward_entry_t";
    x3::rule<reward_entry_1_t_class, ast::reward_entry_1_t> const reward_entry_1_t = "reward_entry_1_t";
    x3::rule<reward_entry_2_t_class, ast::reward_entry_2_t> const reward_entry_2_t = "reward_entry_2_t";


    x3::rule<multi_reward_t_class, ast::multi_reward_t> const multi_reward_t = "multi_reward_t";
    x3::rule<multi_reward_entry_t_class, ast::multi_reward_entry_t> const multi_reward_entry_t = "multi_reward_entry_t";
    x3::rule<multi_reward_entry_1_t_class, ast::multi_reward_entry_1_t> const multi_reward_entry_1_t = "multi_reward_entry_1_t";
    x3::rule<multi_reward_entry_2_t_class, ast::multi_reward_entry_2_t> const multi_reward_entry_2_t = "multi_reward_entry_2_t";

    x3::rule<transition_t_class, ast::transition_t> const transition_t = "transition_t";
    x3::rule<transition_entry_t_class, ast::transition_entry_t> const transition_entry_t = "transition_entry_t";
    x3::rule<transition_entry_1_t_class, ast::transition_entry_1_t> const transition_entry_1_t = "transition_entry_1_t";
    x3::rule<transition_entry_2_t_class, ast::transition_entry_2_t> const transition_entry_2_t = "transition_entry_2_t";
    x3::rule<transition_entry_3_t_class, ast::transition_entry_3_t> const transition_entry_3_t = "transition_entry_3_t";

    x3::rule<observation_t_class, ast::observation_t> const observation_t = "observation_t";
    x3::rule<observation_entry_t_class, ast::observation_entry_t> const observation_entry_t = "observation_entry_t";
    x3::rule<observation_entry_1_t_class, ast::observation_entry_1_t> const observation_entry_1_t = "observation_entry_1_t";
    x3::rule<observation_entry_2_t_class, ast::observation_entry_2_t> const observation_entry_2_t = "observation_entry_2_t";
    x3::rule<observation_entry_3_t_class, ast::observation_entry_3_t> const observation_entry_3_t = "observation_entry_3_t";

    ///////////////////////////////////////////////////////////////////////////
    // Grammar
    ///////////////////////////////////////////////////////////////////////////
    auto const star = +(char_('*'));
    auto const quoted_string = lexeme['"' >> *(char_ - '"') >> '"'];

    auto const vector_of_reals = lexeme[float_ % +blank];
    auto const vector_of_numbers = lexeme[ushort_ % eol];
    auto const vector_of_names = lexeme[quoted_string % +blank];
    auto const vector_stochastic = quoted_string | vector_of_reals;
    auto const number_or_vector_of_names = ushort_ | vector_of_names;
    auto const vector_of_vector_of_reals = lexeme[vector_of_reals % eol];
    auto const vector_of_vector_of_names = lexeme[vector_of_names % eol];
    auto const numbers_or_names = vector_of_numbers | vector_of_vector_of_names;

    auto const identifier_t_def = quoted_string | ushort_;
    auto const identifiers_t_def = lexeme[identifier_t % *blank];

    auto const matrix_t_def = quoted_string | vector_of_vector_of_reals;

    ///////////////////////////////////////////////////////////////////////////////
    // Reward
    ///////////////////////////////////////////////////////////////////////////////
    auto const reward_entry_1_t_def =
        lit("R:") >> identifiers_t >> ':' >> identifier_t >> ':' >> float_;

    auto const reward_entry_2_t_def =
        lit("R:") >> identifiers_t >> ':' >> vector_stochastic;

    auto const reward_entry_t_def = reward_entry_1_t | reward_entry_2_t;

    auto const reward_t_def = +reward_entry_t;


    ///////////////////////////////////////////////////////////////////////////////
    // Multi Reward
    ///////////////////////////////////////////////////////////////////////////////
    auto const multi_reward_entry_1_t_def =
        lit("R:") >> identifiers_t >> ':' >> identifier_t >> ':' >> identifier_t >> ':' >> float_;

    auto const multi_reward_entry_2_t_def =
        lit("R:") >> identifiers_t >> ':' >> identifier_t >> ':' >> vector_stochastic;

    auto const multi_reward_entry_t_def = multi_reward_entry_1_t | multi_reward_entry_2_t;

    auto const multi_reward_t_def = +multi_reward_entry_t;
    ///////////////////////////////////////////////////////////////////////////////
    // Transitions
    ///////////////////////////////////////////////////////////////////////////////
    auto const transition_entry_1_t_def =
        lit("T:") >> identifiers_t >> ':' >> identifier_t >> ':' >> identifier_t >> ':' >> float_;

    auto const transition_entry_2_t_def =
        lit("T:") >> identifiers_t >> ':' >> identifier_t >> ':' >> vector_stochastic;

    auto const transition_entry_3_t_def =
        lit("T:") >> identifiers_t >> ':' >> matrix_t;

    auto const transition_entry_t_def = transition_entry_1_t | transition_entry_2_t | transition_entry_3_t;

    auto const transition_t_def = +transition_entry_t;
    // ///////////////////////////////////////////////////////////////////////////////
    // Observations
    // ///////////////////////////////////////////////////////////////////////////////
    auto const observation_entry_1_t_def =
        lit("O:") >> identifiers_t >> ':' >> identifier_t >> ':' >> identifiers_t >> ':' >> float_;

    auto const observation_entry_2_t_def =
        lit("O:") >> identifiers_t >> ':' >> identifier_t >> ':' >> vector_stochastic;

    auto const observation_entry_3_t_def =
        lit("O:") >> identifiers_t >> ':' >> matrix_t;

    auto const observation_entry_t_def = observation_entry_1_t | observation_entry_2_t | observation_entry_3_t;

    auto const observation_t_def = +observation_entry_t;

    ///////////////////////////////////////////////////////////////////////////////

    auto const mdp_t_def =
        lit("discount") >> ':' >> float_
        >> lit("values") >> ':' >> quoted_string
        >> lit("states") >> ':' >> number_or_vector_of_names
        >> lit("start") >> ':' >> vector_stochastic
        >> lit("actions") >> ':' >> numbers_or_names
        >> transition_t
        >> reward_t;

    auto const mmdp_t_def =
        lit("agents") >> ':' >> number_or_vector_of_names 
        >> lit("discount") >> ':' >> float_
        >> lit("values") >> ':' >> quoted_string
        >> lit("states") >> ':' >> number_or_vector_of_names
        >> lit("start") >> ':' >> vector_stochastic
        >> lit("actions") >> ':' >> numbers_or_names
        >> transition_t
        >> reward_t;

    auto const pomdp_t_def =
        lit("discount") >> ':' >> float_
        >> lit("values") >> ':' >> quoted_string
        >> lit("states") >> ':' >> number_or_vector_of_names
        >> lit("start") >> ':' >> vector_stochastic
        >> lit("actions") >> ':' >> numbers_or_names
        >> lit("observations") >> ':' >> numbers_or_names
        >> transition_t
        >> observation_t
        >> reward_t;

    auto const dpomdp_t_def =
        lit("agents") >> ':' >> number_or_vector_of_names 
        >> lit("discount") >> ':' >> float_
        >> lit("values") >> ':' >> quoted_string
        >> lit("states") >> ':' >> number_or_vector_of_names
        >> lit("start") >> ':' >> vector_stochastic
        >> lit("actions") >> ':' >> numbers_or_names
        >> lit("observations") >> ':' >> numbers_or_names
        >> transition_t
        >> observation_t
        >> reward_t;

    auto const posg_t_def =
        lit("agents") >> ':' >> number_or_vector_of_names 
        >> lit("discount") >> ':' >> float_
        >> lit("values") >> ':' >> quoted_string
        >> lit("states") >> ':' >> number_or_vector_of_names
        >> lit("start") >> ':' >> vector_stochastic
        >> lit("actions") >> ':' >> numbers_or_names
        >> lit("observations") >> ':' >> numbers_or_names
        >> transition_t
        >> observation_t
        >> multi_reward_t;

    BOOST_SPIRIT_DEFINE(identifier_t)
    BOOST_SPIRIT_DEFINE(identifiers_t)
    BOOST_SPIRIT_DEFINE(matrix_t)
    BOOST_SPIRIT_DEFINE(transition_entry_1_t)
    BOOST_SPIRIT_DEFINE(transition_entry_2_t)
    BOOST_SPIRIT_DEFINE(transition_entry_3_t)
    BOOST_SPIRIT_DEFINE(transition_entry_t)
    BOOST_SPIRIT_DEFINE(transition_t)
    BOOST_SPIRIT_DEFINE(observation_entry_1_t)
    BOOST_SPIRIT_DEFINE(observation_entry_2_t)
    BOOST_SPIRIT_DEFINE(observation_entry_3_t)
    BOOST_SPIRIT_DEFINE(observation_entry_t)
    BOOST_SPIRIT_DEFINE(observation_t)
    BOOST_SPIRIT_DEFINE(reward_entry_1_t)
    BOOST_SPIRIT_DEFINE(reward_entry_2_t)
    BOOST_SPIRIT_DEFINE(reward_entry_t)
    BOOST_SPIRIT_DEFINE(reward_t)
    BOOST_SPIRIT_DEFINE(multi_reward_entry_1_t)
    BOOST_SPIRIT_DEFINE(multi_reward_entry_2_t)
    BOOST_SPIRIT_DEFINE(multi_reward_entry_t)
    BOOST_SPIRIT_DEFINE(multi_reward_t)
    BOOST_SPIRIT_DEFINE(mdp_t)
    BOOST_SPIRIT_DEFINE(mmdp_t)
    BOOST_SPIRIT_DEFINE(pomdp_t)
    BOOST_SPIRIT_DEFINE(dpomdp_t)
    BOOST_SPIRIT_DEFINE(posg_t)

    ///////////////////////////////////////////////////////////////////////////
    // Annotation and Error handling
    ///////////////////////////////////////////////////////////////////////////

    // We want these to be annotated with the iterator position.
    // struct dpomdp_inner_class {};

    // struct matrix_t_class {};

    // struct identifier_t_class : x3::annotate_on_success
    // {
    // };
    // struct identifiers_t_class : x3::annotate_on_success
    // {
    // };

    // struct reward_t_class {};
    // struct reward_entry_t_class {};
    // struct reward_entry_1_t_class {};
    // struct reward_entry_2_t_class {};

    // struct transition_t_class {};
    // struct transition_entry_t_class {};
    // struct transition_entry_1_t_class {};
    // struct transition_entry_2_t_class {};
    // struct transition_entry_3_t_class {};

    // struct observation_t_class {};
    // struct observation_entry_t_class {};
    // struct observation_entry_1_t_class {};
    // struct observation_entry_2_t_class {};
    // struct observation_entry_3_t_class {};

    // // We want error-handling only for the start (outermost) dpomdp
    // // rexpr is the same as dpomdp_inner but without error-handling (see error_handler.hpp)
    // struct dpomdp_class : x3::annotate_on_success {};

  } // namespace parser
} // namespace sdm
