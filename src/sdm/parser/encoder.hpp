/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

///////////////////////////////////////////////////////////////////////////////
//  AST processing
///////////////////////////////////////////////////////////////////////////////
namespace sdm
{
  namespace ast
  {

    //! \struct vector_encoder
    //! \brief encodes the input into a vector
    struct vector_encoder : boost::static_visitor<Vector>
    {

      //! \brief the size of the vector vector
      number size;

      vector_encoder(const number size) : boost::static_visitor<Vector>()
      {
        this->size = size;
      }

      //! \param name the way to encode the vector
      //! \brief encodes a string into a vector of probabilities
      Vector operator()(const std::string &name) const
      {
        Vector v(size);
        if (name == "uniform")
        {
          for (state i = 0; i < size; ++i)
          {
            v[i] = 1.0 / size;
          }
        }

        return v;
      }

      //! \param vector the vector
      //! \brief encodes a vector into a vector vector
      Vector operator()(const std::vector<float> &vector) const
      {
        Vector v(size);
        for (state i = 0; i < size; ++i)
        {
          v[i] = vector[i];
        }

        return v;
      }
    };

    //! \struct matrix_encoder
    //! \brief encodes the input into a matrix
    struct matrix_encoder : boost::static_visitor<Matrix>
    {
      number rows, cols;

      matrix_encoder(number rows, number cols) : rows(rows), cols(cols) {}

      //! \param str the way to encode the matrix
      //! \brief encodes a string into a matrix
      Matrix operator()(const std::string &str) const
      {
        number s, s_;
        Matrix m(rows, cols);

        if (str == "uniform")
        {
          for (s = 0; s < rows; ++s)
          {
            for (s_ = 0; s_ < cols; ++s_)
            {
              m(s, s_) = 1.0 / cols;
            }
          }
        }

        else if (str == "identity")
        {
          for (s = 0; s < rows; ++s)
          {
            for (s_ = 0; s_ < cols; ++s_)
            {
              m(s, s_) = s == s_ ? 1.0 : 0.0;
            }
          }
        }

        return m;
      }

      //! \param v the matrix
      //! \brief encodes a matrix into a matrix
      Matrix operator()(const std::vector<std::vector<float>> &v) const
      {
        state s, s_;
        Matrix m(rows, cols);

        for (s = 0; s < rows; ++s)
        {
          for (s_ = 0; s_ < cols; ++s_)
          {
            m(s, s_) = v[s][s_];
          }
        }

        return m;
      }
    };

    //! \struct element_encoder
    //! \brief encodes the input into a number (index)
    struct element_encoder : boost::static_visitor<std::string>
    {
      MultiDiscreteSpace<std::string> md_space_;
      number ag;

      element_encoder(const MultiDiscreteSpace<std::string> &md_space, number ag) : boost::static_visitor<std::string>()
      {
        this->ag = ag;
        this->md_space_ = md_space;
      }

      std::string operator()(number a) const
      {
        return this->md_space_.getSpace(this->ag)->getItem(a);
      }

      std::string operator()(const std::string &a_str) const
      {
        return a_str;
      }
    };

    struct str_visitor
    {
      using result_type = std::string;

      result_type operator()(number v) const { return std::to_string(v); }
      result_type operator()(const std::string &v) { return v; }
    };

    //! \struct joint_element_encoder
    //! \brief encodes the input into a joint element (vector of number)
    class joint_element_encoder
    {
    protected:
      MultiDiscreteSpace<std::string> element_space_;
      DiscreteSpace<std::string> ag_space_;

    public:
      joint_element_encoder(const MultiDiscreteSpace<std::string> &element_space, const DiscreteSpace<std::string> &ag_space)
      {
        this->ag_space_ = ag_space;
        this->element_space_ = element_space;
      }

      std::vector<std::vector<std::string>> encode(const std::vector<identifier_t> &as) const
      {
        std::vector<std::vector<std::string>> a_vec;
        str_visitor str_vis;

        if (as.size() == 1 && boost::apply_visitor(str_vis, as[0]) == "*")
        {
          for (number a = 0; a < this->element_space_.getNumJointItems(); ++a)
          {
            a_vec.push_back(this->element_space_.single2joint(a));
          }
        }
        else
        {
          std::vector<std::string> ja;
          for (agent ag = 0; ag < this->ag_space_.getNumItems(); ++ag)
          {
            element_encoder a_encoder(this->element_space_, ag);
            std::string a_ = boost::apply_visitor(a_encoder, as[ag]);
            ja.push_back(a_);
          }
          a_vec.push_back(ja);
        }
        return a_vec;
      }
    };

    //! \struct state_encoder
    //! \brief encodes the input into a vector of number (vector of states)
    //! "*" -> [0,1,2,3,4,...,n]
    //! "s0" -> [0]
    //! 0 -> [0]
    struct state_encoder : boost::static_visitor<std::vector<std::string>>
    {

      DiscreteSpace<std::string> state_space_;

      state_encoder(const DiscreteSpace<std::string> &state_space) : boost::static_visitor<std::vector<std::string>>()
      {
        this->state_space_ = state_space;
      }

      std::vector<std::string> operator()(number s) const
      {
        return {this->state_space_.getItem(s)};
      }

      std::vector<std::string> operator()(const std::string &s_str) const
      {
        std::vector<std::string> st_ptr;
        if (s_str == "*")
        {
          for (number s = 0; s < this->state_space_.getNumItems(); ++s)
          {
            st_ptr.push_back(this->state_space_.getItem(s));
          }
        }
        else
        {
          st_ptr.push_back(s_str);
        }

        return st_ptr;
      }
    };

    //! \struct discrete_space_encoder
    //! \brief encodes the input into a discrete space class
    struct discrete_space_encoder : boost::static_visitor<DiscreteSpace<std::string>>
    {
      DiscreteSpace<std::string> operator()(number ag) const
      {
        std::vector<std::string> values;
        for (int i = 0; i < ag; i++)
        {
          values.push_back(std::to_string(i));
        }
        return DiscreteSpace<std::string>(values);
      }

      DiscreteSpace<std::string> operator()(const std::vector<std::string> &ags) const
      {
        return DiscreteSpace<std::string>(ags);
      }
    };

    //! \struct multi_discrete_space_encoder
    //! \brief encodes the input into a multi discrete space class
    struct multi_discrete_space_encoder : boost::static_visitor<MultiDiscreteSpace<std::string>>
    {
      MultiDiscreteSpace<std::string> operator()(const std::vector<number> &dim_spaces) const
      {
        std::vector<std::vector<std::string>> vv_values;
        for (auto &dim : dim_spaces)
        {
          std::vector<std::string> values;
          for (int i = 0; i < dim; i++)
          {
            values.push_back(std::to_string(i));
          }
          vv_values.push_back(values);
        }
        MultiDiscreteSpace<std::string> act_space(vv_values);
        return act_space;
      }

      MultiDiscreteSpace<std::string> operator()(const std::vector<std::vector<std::string>> &names) const
      {
        MultiDiscreteSpace<std::string> act_space(names);
        return act_space;
      }
    };

    //! \struct state_transition_encoder
    //! \brief used to encode state transition dynamics (i.e. StateDynamics class)
    struct state_transition_encoder : boost::static_visitor<>
    {
      std::vector<Matrix> t_model;

      DiscreteSpace<std::string> st_space_;
      DiscreteSpace<std::string> ag_space_;
      MultiDiscreteSpace<std::string> act_space_;

      state_transition_encoder(const DiscreteSpace<std::string> &st_space, const DiscreteSpace<std::string> &ag_space, const MultiDiscreteSpace<std::string> &act_space) : boost::static_visitor<>(), st_space_(st_space), ag_space_(ag_space), act_space_(act_space)
      {
        for (number a = 0; a < this->act_space_.getNumJointItems(); ++a)
        {
          auto t = Matrix(this->st_space_.getNumItems(), this->st_space_.getNumItems());

          for (number x = 0; x < this->st_space_.getNumItems(); ++x)
          {
            for (number y = 0; y < this->st_space_.getNumItems(); ++y)
            {
              t(x, y) = 0.0;
            }
          }
          this->t_model.push_back(t);
        }
      }

      void operator()(const transition_entry_3_t &t3)
      {

        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<std::vector<std::string>> ja = ja_encoder.encode(t3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumItems(), this->st_space_.getNumItems());
        Matrix prob = boost::apply_visitor(m_encoder, t3.transitions);

        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          this->t_model[ind_ja] = prob;
        }
      }

      void operator()(const transition_entry_2_t &t2)
      {
        vector_encoder bl_encoder(this->st_space_.getNumItems());
        Vector prob = boost::apply_visitor(bl_encoder, t2.probabilities);

        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<std::vector<std::string>> ja = ja_encoder.encode(t2.jaction);

        state_encoder x_encoder(this->st_space_);
        std::vector<std::string> x_space = boost::apply_visitor(x_encoder, t2.current_state);

        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          for (std::string &x : x_space)
          {
            number ind_state = st_space_.getItemIndex(x);
            for (number y = 0; y < this->st_space_.getNumItems(); ++y)
            {
              this->t_model[ind_ja](ind_state, y) = prob[y];
            }
          }
        }
      }

      void operator()(const transition_entry_1_t &t1)
      {
        state_encoder s_encoder(this->st_space_);
        auto y_space = boost::apply_visitor(s_encoder, t1.next_state);
        auto x_space = boost::apply_visitor(s_encoder, t1.current_state);
        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        auto ja = ja_encoder.encode(t1.jaction);
        double prob = t1.probability;
        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          for (std::string &x : x_space)
          {
            number ind_state = st_space_.getItemIndex(x);
            for (std::string &y : y_space)
            {
              number ind_state_y = st_space_.getItemIndex(y);
              this->t_model[ind_ja](ind_state, ind_state_y) = prob;
            }
          }
        }
      }
    };

    class state_dynamics_encoder
    {
    protected:
      DiscreteSpace<std::string> state_space_;
      DiscreteSpace<std::string> agent_space_;
      MultiDiscreteSpace<std::string> action_space_;

    public:
      state_dynamics_encoder(const DiscreteSpace<std::string> &state_space, const DiscreteSpace<std::string> &agent_space, const MultiDiscreteSpace<std::string> &action_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
      }

      StateDynamics encode(const transition_t &transits)
      {
        StateDynamics s_dyn(this->action_space_.getNumJointItems(), this->state_space_.getNumItems());
        state_transition_encoder state_d_encoder(this->state_space_, this->agent_space_, this->action_space_);
        for (transition_entry_t const &tr : transits)
        {
          boost::apply_visitor(state_d_encoder, tr);
        }
        s_dyn.setTransitions(state_d_encoder.t_model);
        return s_dyn;
      }
    };

    //! \struct observation_transition_encoder
    //! \brief encodes the input into a dynamic class
    struct observation_transition_encoder : boost::static_visitor<>
    {
      std::vector<Matrix> o_model;

      DiscreteSpace<std::string> st_space_;
      DiscreteSpace<std::string> ag_space_;
      MultiDiscreteSpace<std::string> act_space_;
      MultiDiscreteSpace<std::string> obs_space_;

      observation_transition_encoder(const DiscreteSpace<std::string> &st_space, const DiscreteSpace<std::string> &ag_space, const MultiDiscreteSpace<std::string> &act_space, const MultiDiscreteSpace<std::string> &obs_space) : boost::static_visitor<>(), st_space_(st_space), ag_space_(ag_space), act_space_(act_space), obs_space_(obs_space)
      {
        for (number a = 0; a < this->act_space_.getNumJointItems(); ++a)
        {
          auto o = Matrix(this->st_space_.getNumItems(), this->obs_space_.getNumJointItems());

          for (number x = 0; x < this->st_space_.getNumItems(); ++x)
          {
            for (number z = 0; z < this->obs_space_.getNumJointItems(); ++z)
            {
              o(x, z) = 0.0;
            }
          }

          this->o_model.push_back(o);
        }
      }

      void operator()(const observation_entry_3_t &z3)
      {
        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<std::vector<std::string>> ja = ja_encoder.encode(z3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumItems(), this->obs_space_.getNumJointItems());
        Matrix prob = boost::apply_visitor(m_encoder, z3.probabilities);

        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          this->o_model[ind_ja] = prob;
        }
      }

      void operator()(const observation_entry_2_t &z2)
      {
        vector_encoder bl_encoder(this->obs_space_.getNumJointItems());
        Vector prob = boost::apply_visitor(bl_encoder, z2.probabilities);

        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        auto ja = ja_encoder.encode(z2.jaction);

        state_encoder s_encoder(this->st_space_);
        auto s_space = boost::apply_visitor(s_encoder, z2.next_state);

        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          for (number z = 0; z < this->obs_space_.getNumJointItems(); ++z)
          {
            for (std::string &y : s_space)
            {
              number ind_state_y = st_space_.getItemIndex(y);
              this->o_model[ind_ja](ind_state_y, z) = prob[z];
            }
          }
        }
      }

      void operator()(const observation_entry_1_t &z1)
      {
        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        auto ja = ja_encoder.encode(z1.jaction);

        joint_element_encoder jz_encoder(this->obs_space_, this->ag_space_);
        auto jz = jz_encoder.encode(z1.next_observation);

        state_encoder s_encoder(this->st_space_);
        auto y_space = boost::apply_visitor(s_encoder, z1.next_state);

        double prob = z1.probability;
        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = act_space_.joint2single(a);
          for (std::vector<std::string> &z : jz)
          {
            number ind_jobs = obs_space_.joint2single(z);
            for (std::string &y : y_space)
            {
              number ind_state_y = st_space_.getItemIndex(y);
              this->o_model[ind_ja](ind_state_y, ind_jobs) = prob;
            }
          }
        }
      }
    };

    class obs_dynamics_encoder
    {
    protected:
      DiscreteSpace<std::string> state_space_;
      DiscreteSpace<std::string> agent_space_;
      MultiDiscreteSpace<std::string> action_space_;
      MultiDiscreteSpace<std::string> obs_space_;

    public:
      obs_dynamics_encoder(const DiscreteSpace<std::string> &state_space, const DiscreteSpace<std::string> &agent_space, const MultiDiscreteSpace<std::string> &action_space, const MultiDiscreteSpace<std::string> &obs_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
        this->obs_space_ = obs_space;
      }

      ObservationDynamics encode(const observation_t &observs, StateDynamics st_dyn)
      {
        ObservationDynamics dynamics(this->action_space_.getNumJointItems(), this->obs_space_.getNumJointItems(), this->state_space_.getNumItems());
        observation_transition_encoder obs_d_encoder(this->state_space_, this->agent_space_, this->action_space_, this->obs_space_);

        for (observation_entry_t const &obs : observs)
        {
          boost::apply_visitor(obs_d_encoder, obs);
        }
        dynamics.setObservations(obs_d_encoder.o_model);

        for (number u = 0; u < this->action_space_.getNumJointItems(); ++u)
        {
          for (number x = 0; x < this->state_space_.getNumItems(); ++x)
          {
            for (number y = 0; y < this->state_space_.getNumItems(); ++y)
            {
              for (number z = 0; z < this->obs_space_.getNumJointItems(); ++z)
              {
                dynamics.setDynamics(x, u, z, y, st_dyn.getTransitionProbability(x, u, y) * obs_d_encoder.o_model[u](y, z));
              }
            }
          }
        }
        return dynamics;
      }
    };

    struct reward_encoder : boost::static_visitor<>
    {
      DiscreteSpace<std::string> state_space_;
      DiscreteSpace<std::string> ag_space_;
      MultiDiscreteSpace<std::string> action_space_;
      Reward *rewards_;

      reward_encoder(const DiscreteSpace<std::string> &state_space, const DiscreteSpace<std::string> &ag_space, const MultiDiscreteSpace<std::string> &action_space, Reward *rewards) : boost::static_visitor<>()
      {
        this->state_space_ = state_space;
        this->ag_space_ = ag_space;
        this->action_space_ = action_space;
        this->rewards_ = rewards;
      }

      void operator()(const reward_entry_1_t &r1)
      {
        double r = r1.reward;
        state_encoder s_encoder(this->state_space_);
        std::vector<std::string> s_ptr = boost::apply_visitor(s_encoder, r1.state);

        joint_element_encoder ja_encoder(this->action_space_, this->ag_space_);
        std::vector<std::vector<std::string>> ja = ja_encoder.encode(r1.jaction);

        for (std::string &s : s_ptr)
        {
          number ind_state = state_space_.getItemIndex(s);
          for (std::vector<std::string> &a : ja)
          {
            number ind_ja = action_space_.joint2single(Joint<std::string>(a));
            this->rewards_->setReward(ind_state, ind_ja, r);
          }
        }
      }

      void operator()(const reward_entry_2_t &r2)
      {
        vector_encoder bl_encoder(this->state_space_.getNumItems());
        Vector v = boost::apply_visitor(bl_encoder, r2.rewards);

        joint_element_encoder ja_encoder(this->action_space_, this->ag_space_);
        std::vector<std::vector<std::string>> ja = ja_encoder.encode(r2.jaction);

        for (std::vector<std::string> &a : ja)
        {
          number ind_ja = action_space_.joint2single(a);
          this->rewards_->setReward(ind_ja, v);
        }
      }
    };

    class rewards_encoder
    {
    protected:
      DiscreteSpace<std::string> state_space_;
      MultiDiscreteSpace<std::string> action_space_;
      DiscreteSpace<std::string> agent_space_;

    public:
      rewards_encoder(const DiscreteSpace<std::string> &state_space, const DiscreteSpace<std::string> &agent_space, const MultiDiscreteSpace<std::string> &action_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
      }

      Reward encode(const reward_t &r)
      {
        Reward rewards(this->action_space_.getNumJointItems(), this->state_space_.getNumItems());
        reward_encoder r_encoder(this->state_space_, this->agent_space_, this->action_space_, &rewards);
        for (reward_entry_t const &rew : r)
        {
          boost::apply_visitor(r_encoder, rew);
        }
        return rewards;
      }
    };

    std::shared_ptr<DiscreteSpace<number>> toNumberedSpace(DiscreteSpace<std::string> &old_sp)
    {
      return std::make_shared<DiscreteSpace<number>>(old_sp.getNumItems());
    }

    std::shared_ptr<MultiDiscreteSpace<number>> toNumberedSpace(MultiDiscreteSpace<std::string> &old_sp)
    {
      std::vector<number> vvtmp;
      for (auto &dsp : old_sp.getSpaces())
      {
        vvtmp.push_back(dsp->getNumItems());
      }
      return std::make_shared<MultiDiscreteSpace<number>>(vvtmp);
    }

    struct dpomdp_encoder : boost::static_visitor<sdm::DiscreteDecPOMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // DecPOMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      std::shared_ptr<sdm::DiscreteDecPOMDP> operator()(dpomdp_t const &ast)
      {
        discrete_space_encoder ds_encoder;
        multi_discrete_space_encoder mds_encoder;

        // Encodes agent space
        DiscreteSpace<std::string> agent_space = boost::apply_visitor(ds_encoder, ast.agent_param);

        // Encodes state space
        DiscreteSpace<std::string> state_space = boost::apply_visitor(ds_encoder, ast.state_param);

        // Encodes action space
        MultiDiscreteSpace<std::string> action_space = boost::apply_visitor(mds_encoder, ast.action_param);

        // Encodes observation space
        MultiDiscreteSpace<std::string> obs_space = boost::apply_visitor(mds_encoder, ast.observation_param);

        // Encodes the reward function
        rewards_encoder rews_encoder(state_space, agent_space, action_space);
        Reward rew__ = rews_encoder.encode(ast.reward_spec);
        auto rew = std::make_shared<Reward>(rew__);

        // Set start probabilities
        vector_encoder bl_encoder(state_space.getNumItems());
        // auto start_distrib = std::make_shared<Vector>(state_space.getNumItems());
        Vector start_distrib__ = boost::apply_visitor(bl_encoder, ast.start_param);
        std::vector<double> start_vector;
        for (sdm::size_t i = 0; i < start_distrib__.size(); i++)
        {
          start_vector.push_back(start_distrib__[i]);
        }
        std::discrete_distribution<number> start_distrib(start_vector.begin(), start_vector.end());

        // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        StateDynamics state_dyn__ = state_dyn_enc.encode(ast.transition_spec);
        auto state_dyn = std::make_shared<StateDynamics>(state_dyn__);

        // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        ObservationDynamics obs_dyn__ = d_encoder.encode(ast.observation_spec, *state_dyn);
        auto obs_dyn = std::make_shared<ObservationDynamics>(obs_dyn__);

        // sdm::DecPOMDP parsed_model(toNumberedSpace(state_space), toNumberedSpace(agent_space), toNumberedSpace(action_space), toNumberedSpace(obs_space), state_dyn, obs_dyn, rew, start_distrib);

        auto parsed_model = std::make_shared<sdm::DiscreteDecPOMDP>(toNumberedSpace(state_space), toNumberedSpace(action_space), toNumberedSpace(obs_space), state_dyn, obs_dyn, rew, start_distrib);

        // sdm::DecPOMDP parsed_model;
        parsed_model->setDiscount(ast.discount_param);
        parsed_model->setCriterion((Criterion)(ast.value_param == "reward"));

#ifdef VERBOSE
        std::cout << "Print model" << std::endl;
        std::cout << parsed_model << std::endl;
#endif
        return parsed_model;
      }
    };

  } // namespace ast
} // namespace sdm
