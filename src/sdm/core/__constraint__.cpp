/*=============================================================================
Copyright (C) 2016-19 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/__constraint__.hpp>

namespace sdm{
  double __constraint__::getCost(state x, action u){
    return std::abs((__reward__::getMinReward() - this->rewards[u][x]) / (__reward__::getMaxReward() - __reward__::getMinReward()));
  }

  double __constraint__::getBound() const{
    return this->bound;
  }
}
