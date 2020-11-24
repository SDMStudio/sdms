/*=============================================================================
Copyright (c) 2016-2018 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <random>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <sdm/types.hpp>
#include <sdm/core/__toidmdp__.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/public/world.hpp>

//!
//! \file     dpomdp.hpp
//! \author   Jilles S. Dibangoye
//! \brief    dpomdp class
//! \version  1.0
//! \date     10 Avril 2016
//!
//! This class provides getter and setter methods for all parameters
//! of a dpomdp problem, including: criterion, discount factor, states,
//! actions, observations, reward and dynamics models.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  //!
  //! \class  dpomdp  dpomdp.hpp
  //!
  //! \param  value   primitive type for rewards and probabilities.
  //! \param  matrix  type used to encode matrices.
  //! \param  vector  type used to encode vectors.
  //!

  class dpomdp : public __toidmdp__
  {
  protected:
    //! \brief type of optimization problem, e.g., reward maximazation or cost minimization.
    bool criterion;

    std::string filename;

    //! \brief factor used to discount rewards (respectively costs) in the future.
    double discount = 1.0;

    //! \brief planning horizon
    number planning_horizon = 0;

    //! \brief initial distribution over the states.
    std::shared_ptr<Vector> start = std::shared_ptr<Vector>(nullptr);

    //! \brief Encoding all pairs (state, observation) in a map (linked to an integer).
    std::unordered_map<number, std::pair<state, observation>> encoding;

    //! \brief initial state generator.
    std::discrete_distribution<number> start_generator;

    //! \brief Next state generator
    std::unordered_map<state, std::unordered_map<action, std::discrete_distribution<number>>> dynamics_generator;

  public:
    //! \fn     dpomdp()
    //! \brief  default constructor
    //! \return instance of dpomdp
    dpomdp();

    //! \fn     dpomdp(char const*)
    //! \brief  constructor
    //! \param  char* filename
    //! \return instance of dpomdp
    //!
    //! This method builds dpomdp problem based on a specified
    //! description file.
    //TODO dpomdp(char const*);

    //! \fn     dpomdp(action, observation)
    //! \brief  constructor of dpomdp instances
    //! \param  action number of joint actions.
    //! \param  observation number of joint observations.
    //! \return instance of dpomdp
    //!
    //! This builds a dpomdp instance starting with the number of joint
    //! actions and observations. These parameters are important to set
    //! multi-dimensional tables of individual actions and observations.
    // dpomdp(action, observation);

    //! \fn     dpomdp(bool, value, const std::vector<vector>&, const std::vector<std::vector<matrix>>&, agent, state, const std::vector<action>&, const std::vector<observation>&)
    //! \brief  constructor of dpomdp instance
    //! \param  bool            criterion
    //! \param  value           discount factor
    //! \param  const std::vector<vector>&         rewards' model
    //! \param  const std::vector<std::vector<matrix>>&        dynamics' model
    //! \param  agent           number of agents
    //! \param  state           number of states
    //! \param  const std::vector<action>&        table of number of individual actions
    //! \param  const std::vector<observation>&    table of number of individual observations
    //! \return instance of dpomdp
    //!
    //! This constructor builds dpomdp instances  assumming knowledge about
    //! all components of a dpomdp problems.
    // dpomdp(bool, double, const std::vector<Vector> &, const std::vector<std::vector<Matrix>> &, agent, state, const std::vector<action> &, const std::vector<observation> &);

    //! \fn     ~dpomdp()
    //! \brief  destructor of the dpomdp instance.
    //!
    //! This destructor delete rewards, dynamics and table of actions and
    //! observations.
    ~dpomdp();

    //! \fn       void setFileName()
    //! \return   void
    void setFileName(std::string);

    std::string getFileName();

    void setBound(double);

    //! \fn       bool getCriterion() const
    //! \brief    Returns the criterion
    //! \return   bool
    bool getCriterion() const;

    //! \fn       void setCriterion(bool)
    //! \brief    Sets the criterion
    void setCriterion(bool);

    //! \fn       value getDiscount() const
    //! \brief    Returns the discount factor
    //! \return   value
    double getDiscount() const;

    //! \fn       void setDiscount(value)
    //! \brief    Sets the discount factor
    void setDiscount(double);

    //! \fn       std::shared_ptr<vector> getStart()
    //! \brief    Returns the initial state-distribution
    //! \return   std::shared_ptr<vector> the initial state-distribution
    std::shared_ptr<Vector> getStart() const;

    //! \fn       void setStart(const std::shared_ptr<Vector>&)
    //! \param    Vector* an initial state-distribution
    //! \brief    Sets the initial state-distribution
    void setStart(const std::shared_ptr<Vector> &);

    void setInternalState(state);

    //! \fn       bool isSound(value)
    //! \param value : tolerance
    //! \brief    returns true if this dpomdp is well-formed
    //! \return   bool
    bool isSound(double = 0.0001) const;

    void setPlanningHorizon(number);

    number getPlanningHorizon();

    void generate(std::string) const;

    virtual state init();

    virtual void execute(action, feedback *);

    void setDynamicsGenerator();

    std::tuple<double, observation, state> getDynamicsGenerator(state, action);

    //! \fn std::string toStdFormat()
    //! \brief Encodes DPOMDP class into a string (standard .dpomdp format).
    //! \return the resulting .dpomdp like string
    std::string toStdFormat() const;

    //! \fn std::string toXML()
    //! \brief Encodes DPOMDP class into a string (XML format).
    //! \return the resulting XML like string
    std::string toXML() const;

    //! \fn std::string toStdFormat()
    //! \brief Encodes DPOMDP class into a string (JSON format).
    //! \return the resulting JSON like string
    std::string toJSON() const;

    void save(std::string, std::string) const;

    /*!
    * \fn std::ostream& operator<<(std::ostream&, const dpomdp&)
    * \brief print the dpomdp
    * \param std::ostream&
    * \param const dpomdp& the individual decision rule to be printed
    * \return std::ostream&
    *
    * This method should produce an output of this form:
    */
    friend std::ostream &operator<<(std::ostream &os, const dpomdp &model)
    {
      os << model.toXML();
      return os;
    }
  };
} // namespace sdm
