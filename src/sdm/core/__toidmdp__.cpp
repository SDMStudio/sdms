/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <sdm/core/__toidmdp__.hpp>

namespace sdm{
  __toidmdp__::__toidmdp__(){}


  bool __toidmdp__::isTransitionIndependent(){
    if( this->isCheckedToidmp ) return this->isToidmdp;

    this->isCheckedToidmp = true;

    if( !this->isJointlyFullyObservable() ){
      this->isToidmdp = false;
      return this->isToidmdp;
    }

    agent i;
    state x,y;
    action ui, u;
    double prob = 0.0;
    observation zxi, zyi;

    for(x=0; x<this->getNumStates(); ++x){
      for(u=0; u<this->getNumActions(); ++u){
        for(y=0; y<this->getNumStates(); ++y){
          prob = this->getDynamics().getTransitionProbability(x, u, y);
          for(i=0; i<this->getNumAgents(); ++i){
            ui = this->getActionSpace().getActionIndex(i, u);
            zxi = this->getObservationSpace().getObservationIndex(i, this->getObservationFromState(x));
            zyi = this->getObservationSpace().getObservationIndex(i, this->getObservationFromState(y));
            this->getDynamics().setTransitionProbability(zxi, ui, zyi, prob, true);
          }
        }
      }
    }

    for(x=0; x<this->getNumStates(); ++x){
      for(u=0; u<this->getNumActions(); ++u){
        for(y=0; y<this->getNumStates(); ++y){
          prob = this->getDynamics().getTransitionProbability(x, u, y);
          for(i=0; i<this->getNumAgents(); ++i){
            ui = this->getActionSpace().getActionIndex(i, u);
            zxi = this->getObservationSpace().getObservationIndex(i, this->getObservationFromState(x));
            zyi = this->getObservationSpace().getObservationIndex(i, this->getObservationFromState(y));
            prob /= this->getDynamics().getTransitionProbability(zxi, ui, zyi);
          }

          if( prob != 1.0 ){
            this->isToidmdp = false;
            return this->isToidmdp;
          }
        }
      }
    }

    for(auto p : this->state2observation){
      this->observation2state.emplace(p.second, p.first);
    }

    this->isToidmdp = true;
    return this->isToidmdp;
  }

  state __toidmdp__::getStateFromObservation(observation z){
    return this->observation2state.at(z);
  }

  double __toidmdp__::getAgentDynamics(agent ag, observation z, action u, observation z_) const{
    return this->idynamics.at(ag)->getDynamics(z,u,z_,z_);
  }

  const std::shared_ptr<Vector>& __toidmdp__::getBelief(agent ag) const{
    return this->ibeliefs.at(ag);
  }
}
