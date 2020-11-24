/*=============================================================================
Copyright (C) 2018 Jilles Steeve Dibangoye
==============================================================================*/
#include <sdm/core/__dmdp__.hpp>

namespace sdm
{

  __dmdp__::__dmdp__(){}

  bool __dmdp__::isJointlyFullyObservable()
  {
    if (this->isCheckedDmdp)
      return this->isDmdp;

    this->isCheckedDmdp = true;

    for (action u = 0; u < this->getNumActions(); ++u)
    {
      for (state x = 0; x < this->getNumStates(); ++x)
      {

        if (u == 0)
        {
          for (observation z = 0; z < this->getNumObservations(); ++z)
          {
            if (this->getDynamics().getObservationProbability(u, z, x) == 1.0)
              this->state2observation.emplace(x, z);
          }
        }

        if (this->state2observation.find(x) == this->state2observation.end() or this->getDynamics().getObservationProbability(u, this->state2observation.at(x), x) != 1.0)
        {
          this->isDmdp = false;
          return this->isDmdp;
        }
      }
    }

    this->isDmdp = true;
    return this->isDmdp;
  }

  observation __dmdp__::getObservationFromState(state x)
  {
    return this->state2observation.at(x);
  }



  // Getter et Setter
  agent __dmdp__::getNumAgents() const
  {
    return this->getAgentSpace().getNumAgents();
  }

  state __dmdp__::getNumStates() const
  {
    return this->getStateSpace().getNumStates();
  }

  observation __dmdp__::getNumObservations() const
  {
    return this->getObservationSpace().getNumObservations();
  }

  action __dmdp__::getNumActions() const
  {
    return this->getActionSpace().getNumActions();
  }

  __state__ __dmdp__::getStateSpace() const
  {
    return this->state_space_;
  }

  void __dmdp__::setStateSpace(__state__ state_space)
  {
    this->state_space_ = state_space;
  }

  __agent__ __dmdp__::getAgentSpace() const
  {
    return this->agent_space_;
  }

  void __dmdp__::setAgentSpace(__agent__ agent_space)
  {
    this->agent_space_ = agent_space;
  }

  __action__ __dmdp__::getActionSpace() const
  {
    return this->action_space_;
  }

  void __dmdp__::setActionSpace(__action__ action_space)
  {
    this->action_space_ = action_space;
  }

  __observation__ __dmdp__::getObservationSpace() const
  {
    return this->observation_space_;
  }

  void __dmdp__::setObservationSpace(__observation__ observation_space)
  {
    this->observation_space_ = observation_space;
  }

  __reward__ __dmdp__::getReward() const
  {
    return this->reward_;
  }

  void __dmdp__::setReward(__reward__ reward_fct)
  {
    this->reward_ = reward_fct;
  }

  __dynamics__ __dmdp__::getDynamics() const
  {
    return this->dynamics_;
  }

  void __dmdp__::setDynamics(__dynamics__ dynamics)
  {
    this->dynamics_ = dynamics;
  }

} // namespace sdm
