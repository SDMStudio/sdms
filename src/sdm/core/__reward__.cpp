/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/__reward__.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm{
  void __reward__::initReward(action num_jactions, state num_states){
    action a;
    for(a=0; a<num_jactions; ++a){
      auto v = Vector( num_states );
      v.init(0.0);
      this->rewards.push_back( v );
    }
  }

  double __reward__::getReward(state s, action a) const{
    return this->rewards[a][s];
  }

  const Vector& __reward__::getReward(action a) const{
    return this->rewards[a];
  }

  const std::vector<Vector>& __reward__::getReward() const{
    return this->rewards;
  }

  void __reward__::setReward(action a, const Vector& v){
    auto r = v.min();
    this->min = std::min(r, this->min);

    r = v.max();
    this->max = std::max(r, this->max);

    this->rewards[a] = v;
  }

  void __reward__::setReward(state s, action a, double r){
    this->min = std::min(r, this->min);
    this->max = std::max(r, this->max);
    this->rewards[a][s] = r;
  }

  double __reward__::getMaxReward(){
    return this->max;
  }

  double __reward__::getMinReward(){
    return this->min;
  }

}
