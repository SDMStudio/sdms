/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <sdm/core/dpomdp.hpp>
#include <sdm/common.hpp>

#include <sstream>
#include <regex>

namespace sdm
{

  dpomdp::dpomdp() {

  }

  // dpomdp::dpomdp(action number_jactions, observation number_jobservations)
  // {
  //   this->number_jactions = number_jactions;
  //   this->rewards.resize(number_jactions);
  //   this->number_jobservations = number_jobservations;
  //   for (std::size_t i = 0; i < number_jactions; i++)
  //     this->dynamics.push_back(std::vector<Matrix>(number_jobservations));
  // }

  // dpomdp::dpomdp(bool criterion, double discount,
  //                const std::vector<Vector> &rewards,
  //                const std::vector<std::vector<Matrix>> &dynamics,
  //                agent number_agents, state number_states,
  //                const std::vector<action> &number_actions,
  //                const std::vector<observation> &number_observations) : criterion(criterion), discount(discount)
  // {
  //   this->rewards = rewards;
  //   this->dynamics = dynamics;
  //   this->number_agents = number_agents;
  //   this->states_.setNumStates(number_states);
  //   this->number_actions = number_actions;
  //   this->number_observations = number_observations;
  //   this->number_jactions = 1;

  //   agent i;
  //   for (i = 0; i < this->number_agents; ++i)
  //     this->number_jactions *= number_actions[i];
  //   this->generateJointActions(this->number_agents);

  //   this->number_jobservations = 1;
  //   for (i = 0; i < number_agents; ++i)
  //     this->number_jobservations *= number_observations[i];
  //   this->generateJointObservations(this->number_agents);
  // }

  dpomdp::~dpomdp()
  {
  }

  void dpomdp::setFileName(std::string filename)
  {
    this->filename = filename;
  }

  std::string dpomdp::getFileName()
  {
    return this->filename;
  }

  bool dpomdp::getCriterion() const
  {
    return criterion;
  }

  void dpomdp::setCriterion(bool criterion)
  {
    this->criterion = criterion;
  }

  void dpomdp::setBound(double bound)
  {
    this->bound = std::min(1.0 / (bound * (1.0 - this->discount)), 1.0);
  }

  double dpomdp::getDiscount() const
  {
    return discount;
  }

  void dpomdp::setDiscount(double discount)
  {
    this->discount = discount;
  }

  std::shared_ptr<Vector> dpomdp::getStart() const
  {
    return start;
  }

  void dpomdp::setStart(const std::shared_ptr<Vector> &start)
  {
    this->start = start;
    std::cout << "Warning : Belief not saved !!\n";
    // common::saveBeliefs(this->start);
  }

  state dpomdp::init()
  {
    this->internal = this->start_generator(sdm::common::global_urng());
    return this->internal;
  }

  void dpomdp::execute(action u, feedback *f)
  {
    assert(f != nullptr);

    state x;
    double r;
    observation z;

    std::tie(r, z, x) = this->getDynamicsGenerator(this->internal, u);

    f->setReward(r);
    this->internal = x;
    f->setObservation(z);
    f->setState(this->internal);

    assert(this->internal < this->getNumStates());
    assert(f->getObservation() < this->getNumObservations());
  }

  void dpomdp::setInternalState(state x)
  {
    this->internal = x;
  }

  bool dpomdp::isSound(double tolerance) const
  {
    //   // Check the initial state-distribution
    //   if (std::abs(this->start->sum() - 1) > tolerance)
    //     return false;

    //   // Check the dynamics model
    //   action a;
    //   observation z;
    //   number size, r;
    //   for (a = 0; a < this->number_jactions; ++a)
    //   {
    //     auto _sum_ = this->dynamics[a][0];
    //     for (z = 1; z < this->number_jobservations; ++z)
    //       _sum_ += this->dynamics[a][z];

    //     size = _sum_.rows();
    //     for (r = 0; r < size; ++r)
    //       if (std::abs(_sum_.row(r).sum() - 1) > tolerance)
    //         return false;
    //   }
    std::cout << "[dpomdp.cpp] isSound() not implemented\n";
    return true;
  }

  void dpomdp::setDynamicsGenerator()
  {
    number i;
    action a;
    state x, y;
    observation z;

    for (i = 0, y = 0; y < this->getNumStates(); ++y)
      for (z = 0; z < this->getNumObservations(); ++z, ++i)
        this->encoding.emplace(i, std::make_pair(y, z));

    for (x = 0; x < this->getNumStates(); ++x)
    {
      this->dynamics_generator.emplace(x, std::unordered_map<action, std::discrete_distribution<number>>());
      for (a = 0; a < this->getNumActions(); ++a)
      {
        std::vector<double> v;
        for (y = 0; y < this->getNumStates(); ++y)
          for (z = 0; z < this->getNumObservations(); ++z)
            v.push_back(this->getDynamics().getDynamics(x, a, z, y));

        this->dynamics_generator[x].emplace(a, std::discrete_distribution<number>(v.begin(), v.end()));
      }
    }

    std::vector<double> v;
    for (state x = 0; x < this->getNumStates(); ++x)
      v.push_back((*this->start)[x]);
    this->start_generator = std::discrete_distribution<number>(v.begin(), v.end());
  }

  std::tuple<double, observation, state> dpomdp::getDynamicsGenerator(state x, action a)
  {
    state y;
    observation z;
    std::tie(y, z) = this->encoding[this->dynamics_generator[x][a](sdm::common::global_urng())];
    return std::make_tuple(this->getReward().getReward(x, a), z, y);
  }

  void dpomdp::setPlanningHorizon(number planning_horizon)
  {
    this->planning_horizon = planning_horizon;
  }

  number dpomdp::getPlanningHorizon()
  {
    return this->planning_horizon;
  }

  std::string dpomdp::toStdFormat() const
  {
    std::ostringstream res;

    res << "agents: " << this->getNumAgents() << std::endl;
    res << "discount: " << this->getDiscount() / 1.0 << std::endl;
    res << "values: \"reward\"" << std::endl;
    res << "states: " << this->getNumStates() << std::endl;
    res << "start: \"uniform\"" << std::endl;
    res << "actions: \n"
        << this->getActionSpace().getNumActions(0) << "\n"
        << this->getActionSpace().getNumActions(1) << std::endl;
    res << "observations: \n"
        << this->getObservationSpace().getNumObservations(0) << "\n"
        << this->getObservationSpace().getNumObservations(1) << std::endl;

    for (state x = 0; x < this->getNumStates(); ++x)
    {
      for (action u = 0; u < this->getNumActions(); u++)
      {
        auto ja = this->getActionSpace().getJointAction(u);
        auto u1 = ja->getIndividualItem(0);
        auto u2 = ja->getIndividualItem(1);
        for (state y = 0; y < this->getNumStates(); ++y)
        {
          res << "T: " << u1 << " " << u2 << " : " << x << " : " << y << " : " << this->getDynamics().getTransitionProbability(x, u, y) << std::endl;
        }
      }
    }

    for (state y = 0; y < this->getNumStates(); ++y)
    {
      for (action u = 0; u < this->getNumActions(); u++)
      {
        auto ja = this->getActionSpace().getJointAction(u);
        auto u1 = ja->getIndividualItem(0);
        auto u2 = ja->getIndividualItem(1);
        for (observation z = 0; z < this->getNumObservations(); ++z)
        {
          auto jz = this->getObservationSpace().getJointObservation(z);
          auto z1 = jz->getIndividualItem(0);
          auto z2 = jz->getIndividualItem(1);
          res << "O: " << u1 << " " << u2 << " : " << y << " : " << z1 << " " << z2 << " : " << this->getDynamics().getObservationProbability(u, z, y) << std::endl;
        }
      }
    }

    for (state x = 0; x < this->getNumStates(); ++x)
    {
      for (action u = 0; u < this->getNumActions(); u++)
      {
        auto ja = this->getActionSpace().getJointAction(u);
        auto u1 = ja->getIndividualItem(0);
        auto u2 = ja->getIndividualItem(1);
        res << "R: " << u1 << " " << u2 << " : " << x << " : " << this->getReward().getReward(x, u) << std::endl;
      }
    }
    return res.str();
  }

  std::string dpomdp::toXML() const
  {
    std::ostringstream res;

    agent ag;
    res << "<dpomdp>" << std::endl;
    res << "\t<preamble>" << std::endl;
    res << "\t\t<soundness>" << this->isSound() << "</soundness>" << std::endl;
    res << "\t\t<agents>" << this->getNumAgents() << "</agents>" << std::endl;
    res << "\t\t<discount>" << this->getDiscount() << "</discount>" << std::endl;
    res << "\t\t<states>" << this->getNumStates() << "</states>" << std::endl;
    res << "\t\t<start>" << *(this->getStart()) << "</start>" << std::endl;

    res << "\t\t<actions>" << std::endl;
    for (ag = 0; ag < this->getNumAgents(); ++ag)
      res << "\t\t\t<agent id=\"" << ag << "\">" << this->getActionSpace().getNumActions(ag) << "</agent>" << std::endl;
    res << "\t\t</actions>" << std::endl;

    res << "\t\t<observations>" << std::endl;
    for (ag = 0; ag < this->getNumAgents(); ++ag)
      res << "\t\t\t<agent id=\"" << ag << "\">" << this->getObservationSpace().getNumObservations(ag) << "</agent>" << std::endl;
    res << "\t\t</observations>" << std::endl;

    res << "\t</preamble>" << std::endl;
    res << "\t<param>" << std::endl;

    action ja;
    res << "\t\t<reward>" << std::endl;
    for (ja = 0; ja < this->getActionSpace().getNumActions(); ++ja)
    {
      auto u1 = this->getActionSpace().getActionName(0, this->getActionSpace().getActionIndex(0, ja));
      auto u2 = this->getActionSpace().getActionName(1, this->getActionSpace().getActionIndex(1, ja));
      res << "\t\t\t<reward-entry joint-action=\"" << u1 << ", " << u2 << "\" >" << std::endl;
      res << "\t\t\t\t" << this->getReward().getReward(ja) << std::endl;
      res << "\t\t\t</reward-entry>" << std::endl;
    }
    res << "\t\t</reward>" << std::endl;

    observation jz;
    res << "\t\t<dynamics>" << std::endl;
    for (ja = 0; ja < this->getNumActions(); ++ja)
      for (jz = 0; jz < this->getNumObservations(); ++jz)
      {
        res << "\t\t\t<dynamics-entry jaction=\"" << ja << "\" jobservation=\"" << jz << "\">" << std::endl;
        res << "\t\t\t\t" << this->getDynamics().getDynamics(ja, jz) << std::endl;
        res << "\t\t\t</dynamics-entry>" << std::endl;
      }
    res << "\t\t</dynamics>" << std::endl;

    res << "\t</param>" << std::endl;
    res << "</dpomdp>" << std::endl;

    return res.str();
  }

  std::string dpomdp::toJSON() const
  {
    std::cout << "toJSON : Not implemented method" << std::endl;
    return "Not implemented method";
  }

  void dpomdp::generate(std::string filename) const
  {
    std::ofstream myfile;
    if (regex_match(filename, std::regex(".*\\.json$")) || regex_match(filename, std::regex(".*\\.JSON$")))
    {
      myfile = std::ofstream(filename);
      myfile << this->toJSON();
    }
    else if (regex_match(filename, std::regex(".*\\.xml$")) || regex_match(filename, std::regex(".*\\.XML$")))
    {
      myfile = std::ofstream(filename);
      myfile << this->toXML();
    }
    else if (regex_match(filename, std::regex(".*\\.dpomdp$")) || regex_match(filename, std::regex(".*\\.DPOMDP$")))
    {
      myfile = std::ofstream(filename);
      myfile << this->toStdFormat();
    }
    else
    {
      myfile = std::ofstream(filename + ".dpomdp");
      myfile << this->toStdFormat();
    }
    myfile.close();
  }
} // namespace sdm
