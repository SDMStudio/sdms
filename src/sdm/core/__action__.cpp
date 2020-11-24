
#include <iostream>

#include <sdm/core/__action__.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/utils/decision_rules/joint_action.hpp>

namespace sdm
{

  __action__::~__action__()
  {
    for (std::vector<bimap>::iterator it = this->action_names_bimap.begin(); it != this->action_names_bimap.end(); ++it)
    {
      it->erase(it->begin(), it->end());
    }
  }

  action __action__::getNumActions() const
  {
    return number_jactions;
  }

  void __action__::setNumActions(action number_jactions)
  {
    this->number_jactions = number_jactions;
  }

  void __action__::setNumActions(const std::vector<std::vector<std::string>> &action_names)
  {
    this->number_jactions = 1;
    agent ag, idx, num_agents = action_names.size();

    for (ag = 0; ag < num_agents; ++ag)
    {
      idx = action_names[ag].size();
      this->number_jactions *= idx;
      this->number_actions.push_back(idx);
      this->action_names_bimap.push_back(bimap());
    }

    for (ag = 0; ag < num_agents; ++ag)
      for (idx = 0; idx < this->number_actions[ag]; ++idx)
        this->action_names_bimap[ag].insert(name2index(action_names[ag][idx], idx));

    this->generateJointActions(num_agents);
  }

  void __action__::setNumActions(const std::vector<action> &action_numbers)
  {
    this->number_jactions = 1;
    agent ag, num_agents = action_numbers.size();

    for (ag = 0; ag < num_agents; ++ag)
    {
      this->number_jactions *= action_numbers[ag];
      this->number_actions.push_back(action_numbers[ag]);
    }

    this->generateJointActions(num_agents);
  }

  action __action__::getNumActions(agent i) const
  {
    return this->number_actions[i];
  }

  void __action__::setNumActions(agent i, action number_actions)
  {
    this->number_actions[i] = number_actions;
  }

  action __action__::getJointActionIndex(std::vector<action> const &ja) const
  {
    return joint_action::getJointItemIdx(ja);
  }

  action __action__::getJointActionIndex(joint_action *ja) const
  {
    return joint_action::getJointItemIdx(ja);
  }

  joint_action *__action__::getJointAction(action idx)
  {
    return static_cast<joint_action *>(joint_action::getJointItem(idx));
  }

  action __action__::getActionIndex(agent ag, const std::string &name) const
  {
    return this->action_names_bimap.empty() ? 0 : this->action_names_bimap[ag].left.at(name);
  }

  action __action__::getActionIndex(agent ag, action ja)
  {
    return getJointAction(ja)->getIndividualItem(ag);
  }

  std::string __action__::getActionName(agent ag, action u) const
  {
    return this->action_names_bimap.empty() ? std::to_string(u) : this->action_names_bimap[ag].right.at(u);
  }

  // TODO change this in order to make generated joint_action a member of __action__ 
  void __action__::generateJointActions(agent number_agents)
  {
    agent ag;
    std::vector<agent> v_agents;
    for (ag = 0; ag < number_agents; ++ag)
      v_agents.push_back(ag);

    //! generator of variations for joint actions
    variations<std::vector<agent>, joint_action> jaction_generator(v_agents, this->number_actions);
    auto ja = jaction_generator.begin();
    do
    {
      ja = jaction_generator.next();
    } while (ja != nullptr);
  }

} // namespace sdm
