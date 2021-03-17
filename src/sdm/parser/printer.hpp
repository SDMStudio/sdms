/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>

#include <iostream>

///////////////////////////////////////////////////////////////////////////////
//  AST processing
///////////////////////////////////////////////////////////////////////////////
namespace sdm {
  namespace ast{
    struct dpomdp_printer{
      typedef void result_type;

      void operator()(unsigned short short_) const { std::cout << short_; }

      void operator()(std::string const& string_) const { std::cout << string_; }

      template<typename type>
      void operator()(std::vector<type> const& vector_type) const {
        for(auto const& type_ : vector_type) std::cout << ' ' << type_;
      }

      template<typename type>
      void operator()(std::vector<std::vector<type>> const& vector_type) const {
        for(auto const& vector_type_ : vector_type){
          for(auto const& type_ : vector_type_) std::cout << ' ' << type_;
          std::cout << '\n';
        }
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Transition printer
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void operator()(transition_entry_1_t const& tr) const {
        std::cout << "T:";

        for(identifier_t const& id : tr.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, tr.current_state);
        std::cout << ": "; boost::apply_visitor(*this, tr.next_state);
        std::cout << ": " << tr.probability << "\n";
      }

      void operator()(transition_entry_2_t const& tr) const {
        std::cout << "T:";

        for(identifier_t const& id : tr.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, tr.current_state);
        std::cout << ": "; boost::apply_visitor(*this, tr.probabilities);
        std::cout << "\n";
      }

      void operator()(transition_entry_3_t const& tr) const {
        std::cout << "T:";

        for(identifier_t const& id : tr.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": \n";
        boost::apply_visitor(*this, tr.transitions);
        std::cout << "\n";
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Observation printer
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void operator()(observation_entry_1_t const& obs) const {
        std::cout << "O:";

        for(identifier_t const& id : obs.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, obs.next_state);
        std::cout << ":";
        for(identifier_t const& id : obs.next_observation){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": " << obs.probability << "\n";
      }

      void operator()(observation_entry_2_t const& obs) const {
        std::cout << "O:";

        for(identifier_t const& id : obs.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, obs.next_state);
        std::cout << ": "; boost::apply_visitor(*this, obs.probabilities);
        std::cout << "\n";
      }

      void operator()(observation_entry_3_t const& obs) const {
        std::cout << "O:";

        for(identifier_t const& id : obs.jaction){
          std::cout << ' '; boost::apply_visitor(*this, id);
        }

        std::cout << ": \n";
        boost::apply_visitor(*this, obs.probabilities);
        std::cout << "\n";
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Reward printer
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void operator()(reward_entry_1_t const& rew) const {
        std::cout << "R:";

        for(identifier_t const& id : rew.jaction){
          std::cout << ' ';
          boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, rew.state);
        std::cout << ": " << rew.reward << "\n";
      }

      void operator()(reward_entry_2_t const& rew) const {
        std::cout << "R:";

        for(identifier_t const& id : rew.jaction){
          std::cout << ' ';
          boost::apply_visitor(*this, id);
        }

        std::cout << ": "; boost::apply_visitor(*this, rew.rewards);
        std::cout << "\n";
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // dpomdp printer
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void operator()(dpomdp_t const& ast){
        std::cout << "agents: ";
        boost::apply_visitor(*this, ast.agent_param);
        std::cout << "\n";

        std::cout << "value: " << ast.value_param << std::endl;
        std::cout << "discount: " << ast.discount_param << std::endl;

        std::cout << "states:";
        boost::apply_visitor(*this, ast.state_param);
        std::cout << "\n";

        std::cout << "start:";
        boost::apply_visitor(*this, ast.start_param);
        std::cout << "\n";

        std::cout << "actions:";
        boost::apply_visitor(*this, ast.action_param);

        std::cout << "observations:";
        boost::apply_visitor(*this, ast.observation_param);

        for(transition_entry_t const& tr : ast.transition_spec)
          boost::apply_visitor(*this, tr);

        for(observation_entry_t const& obs : ast.observation_spec)
          boost::apply_visitor(*this, obs);

        for(reward_entry_t const& rew : ast.reward_spec)
          boost::apply_visitor(*this, rew);
      }
    };
  }
}
