/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <sdm/core/__state__.hpp>

namespace sdm
{
  __state__::__state__() {}

  __state__::__state__(state number_states) : num_states_(number_states)
  {
  }

  __state__::__state__(const std::vector<std::string> &state_names) : num_states_(state_names.size())
  {
    for (state idx = 0; idx < this->num_states_; ++idx)
      this->state_names_bimap.insert(name2index(state_names[idx], idx));
  }

  state __state__::getNumStates() const
  {
    return this->num_states_;
  }

  void __state__::setNumStates(state number_states)
  {
    this->num_states_ = number_states;
  }

  void __state__::setStatesNames(const std::vector<std::string> &state_names)
  {
    if (this->num_states_ == 0)
    {
      this->setNumStates(state_names.size());
    }
    else
    {
      assert(this->num_states_ == state_names.size());
    }

    for (state idx = 0; idx < this->num_states_; ++idx)
      this->state_names_bimap.insert(name2index(state_names[idx], idx));
  }

  state __state__::getStateIndex(const std::string &name) const
  {
    return this->state_names_bimap.empty() ? 0 : this->state_names_bimap.left.at(name);
  }

  std::string __state__::getStateName(state s) const
  {
    return this->state_names_bimap.empty() ? std::to_string(s) : this->state_names_bimap.right.at(s);
  }

} // namespace sdm
