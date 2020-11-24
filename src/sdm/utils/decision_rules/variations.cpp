/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <iostream>
#include <algorithm>

#include <eigen3/Eigen/Dense>

// #include "../include/dpomdp/dpomdp.hpp"
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/utils/decision_rules/joint_action.hpp>
#include <sdm/utils/decision_rules/joint_observation.hpp>
// #include <sdm/include/utils/decision_rules/joint_decision_rule.hpp>
// #include <sdm/include/utils/decision_rules/individual_policy_tree.hpp>
// #include <sdm/include/utils/decision_rules/individual_decision_rule.hpp>
// #include <sdm/include/utils/decision_rules/joint_decision_rule.hpp>
// #include <sdm/include/utils/decision_rules/delayed_joint_decision_rule.hpp>
// #include <sdm/include/utils/decision_rules/best_response_decision_rule.hpp>
// #include <sdm/include/utils/information_states/history_state/jhistory.hpp>
// #include <sdm/include/utils/decision_rules/light_decision_rule.hpp>


namespace sdm{
  template<typename key, typename output>
  variations<key,output>::variations(){}

  template<typename key, typename output>
  variations<key,output>::variations(const key& parameters, const std::vector<action>& decisions) : decisions(decisions), parameters(parameters) {
    this->setVariation(parameters, decisions);
  }

  template<typename key, typename output>
  void variations<key,output>::setVariation(const key& parameters, const std::vector<action>& decisions){
    this->decisions = decisions;
    this->parameters = parameters;
    auto dimension = this->parameters.size();

    if( this->cbegin != nullptr ){
      delete[] this->cbegin;
      this->cbegin = nullptr;
    }

    this->cbegin = new action[ dimension  ];
    std::fill_n(this->cbegin, dimension, 0);
    this->cend = this->cbegin + (dimension  - 1);
    this->current = this->cend;
  }

  template<typename key, typename output>
  variations<key,output>::~variations(){
    if( cbegin != nullptr ){
      delete[] cbegin;
      cbegin = nullptr;
    }
  }

  template<typename key, typename output>
  output* variations<key,output>::begin(){
    return new output(this->parameters, std::vector<action>(this->parameters.size(), 0));
  }

  template<typename key, typename output>
  output* variations<key,output>::end(){
    return new output(this->parameters, std::vector<action>(this->parameters.size(), this->decisions[0] - 1));
  }

  template<typename key, typename output>
  ptrdiff_t variations<key,output>::index(){
    return this->current - this->cbegin;
  }

  template<typename key, typename output>
  action variations<key,output>::limit(){
    return this->decisions[ index() ]  -  1;
  }

  template<typename key, typename output>
  output* variations<key,output>::next(){
    this->current = this->cend;

    //<! easy case, increase rightmost element
    if( *this->current < this->limit() ) {
      *this->current += 1;
      return new output(this->parameters, std::vector<action>(this->cbegin, this->cbegin + this->parameters.size()));
    }

    //<! find rightmost element to increase and reset right-hand elements
    do{
      *this->current = 0; this->current --;
    }while( this->current >= this->cbegin and *this->current == this->limit() );

    //<! terminate if all elements attained their limits
    if( this->current < this->cbegin ){
       this->current = this->cend;
       return nullptr;
    }

    //<! else, increase the current value, and return the output.
    *this->current += 1;
    return new output(this->parameters, std::vector<action>(this->cbegin, this->cbegin + this->parameters.size()));
  }

  template class variations<std::vector<agent>, joint_action>;
  template class variations<std::vector<agent>, joint_observation>;
  // template class variations<std::vector<observation>, individual_policy_tree>;
  // template class variations<std::vector<std::shared_ptr<Vector>>, light_decision_rule>;
  // template class variations<std::vector<observation_ihistory*>, individual_decision_rule>;
  // template class variations<std::vector<observation_ihistory*>, best_response_decision_rule>;
  // template class variations<std::vector<observation_ihistory*>, deterministic_jdecision_rule>;
  // template class variations<std::vector<observation_ihistory*>, deterministic_uniform_jdecision_rule>;
  // template class variations<std::vector<action_observation_ihistory*>, random_jdecision_rule>;
  // template class variations<std::vector<action_observation_ihistory*>, delayed_joint_decision_rule>;
  // template class variations<std::vector<action_observation_ihistory*>, random_uniform_jdecision_rule>;
}
