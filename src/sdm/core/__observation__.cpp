/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <iostream>

#include <sdm/core/__observation__.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/utils/decision_rules/joint_observation.hpp>


namespace sdm
{

  __observation__::~__observation__()
  {
    for (std::vector<bimap>::iterator it = this->observation_names_bimap.begin(); it != this->observation_names_bimap.end(); ++it)
    {
      it->erase(it->begin(), it->end());
    }
  }

  observation __observation__::getNumObservations() const
  {
    return number_jobservations;
  }

  void __observation__::setNumObservations(const std::vector<std::vector<std::string>> &observation_names)
  {
    this->number_jobservations = 1;
    agent ag, idx, num_agents = observation_names.size();
    for (ag = 0; ag < num_agents; ++ag)
    {
      idx = observation_names[ag].size();
      this->number_jobservations *= idx;
      this->number_observations.push_back(idx);
      this->observation_names_bimap.push_back(bimap());
    }

    for (ag = 0; ag < num_agents; ++ag)
      for (idx = 0; idx < this->number_observations[ag]; ++idx)
        this->observation_names_bimap[ag].insert(name2index(observation_names[ag][idx], idx));

    this->generateJointObservations(num_agents);
  }

  void __observation__::setNumObservations(const std::vector<observation> &observation_numbers)
  {
    this->number_jobservations = 1;
    agent ag, num_agents = observation_numbers.size();
    for (ag = 0; ag < num_agents; ++ag)
    {
      this->number_jobservations *= observation_numbers[ag];
      this->number_observations.push_back(observation_numbers[ag]);
    }

    this->generateJointObservations(num_agents);
  }

  void __observation__::setNumObservations(observation number_jobservations)
  {
    this->number_jobservations = number_jobservations;
  }

  observation __observation__::getNumObservations(agent i) const
  {
    return number_observations[i];
  }

  void __observation__::setNumObservations(agent i, observation number_observations)
  {
    this->number_observations[i] = number_observations;
  }

  observation __observation__::getObservationIndex(agent ag, const std::string &name) const
  {
    return this->observation_names_bimap.empty() ? 0 : this->observation_names_bimap[ag].left.at(name);
  }

  observation __observation__::getObservationIndex(agent ag, observation jz)
  {
    return getJointObservation(jz)->getIndividualItem(ag);
  }

  std::string __observation__::getObservationName(agent ag, observation z) const
  {
    return this->observation_names_bimap.empty() ? std::to_string(z) : this->observation_names_bimap[ag].right.at(z);
  }

  observation __observation__::getJointObservationIndex(joint_observation *jz)
  {
    return joint_observation::getJointItemIdx(jz);
  }

  observation __observation__::getJointObservationIndex(std::vector<observation> const &jz)
  {
    return joint_observation::getJointItemIdx(jz);
  }

  joint_observation *__observation__::getJointObservation(observation jz)
  {
    return static_cast<joint_observation *>(joint_observation::getJointItem(jz));
  }

  void __observation__::generateJointObservations(agent num_agents)
  {
    agent ag;
    std::vector<agent> v_agents;
    for (ag = 0; ag < num_agents; ++ag)
      v_agents.push_back(ag);

    //! generator of variations for joint observations
    variations<std::vector<agent>, joint_observation> jobservation_generator(v_agents, this->number_observations);

    jobservation_generator.begin();
    while (jobservation_generator.next() != nullptr)
      ;
  }
} // namespace sdm
