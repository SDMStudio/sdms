/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/core/dpomdp.hpp>
#include <sdm/core/discrete_space.hpp>
#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/world/decpomdp.hpp>
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
    struct element_encoder : boost::static_visitor<number>
    {
      MultiDiscreteSpace md_space_;
      number ag;

      element_encoder(MultiDiscreteSpace md_space, number ag) : boost::static_visitor<number>()
      {
        this->ag = ag;
        this->md_space_ = md_space;
      }

      number operator()(number a) const
      {
        return a;
      }

      number operator()(const std::string &a_str) const
      {
        return this->md_space_.getElementIndex(this->ag, a_str);
      }
    };

    //! \struct joint_element_encoder
    //! \brief encodes the input into a joint element (vector of number)
    class joint_element_encoder
    {
    protected:
      MultiDiscreteSpace element_space_;
      DiscreteSpace ag_space_;

    public:
      joint_element_encoder(MultiDiscreteSpace element_space, DiscreteSpace ag_space)
      {
        this->ag_space_ = ag_space;
        this->element_space_ = element_space;
      }

      std::vector<number> encode(const std::vector<identifier_t> &as) const
      {
        std::vector<number> a_vec;
        if (as.size() == 1)
        {
          for (number a = 0; a < this->element_space_.getNumJElements(); ++a)
          {
            a_vec.push_back(a);
          }
        }
        else
        {
          std::vector<number> ja;
          for (agent ag = 0; ag < this->ag_space_.getNumElements(); ++ag)
          {
            element_encoder a_encoder(this->element_space_, ag);
            number a_ = boost::apply_visitor(a_encoder, as[ag]);
            ja.push_back(a_);
          }
          a_vec.push_back(this->element_space_.joint2single(ja));
        }
        return a_vec;
      }
    };

    //! \struct state_encoder
    //! \brief encodes the input into a vector of number (vector of states)
    //! "*" -> [0,1,2,3,4,...,n]
    //! "s0" -> [0]
    //! 0 -> [0]
    struct state_encoder : boost::static_visitor<std::vector<number>>
    {

      DiscreteSpace state_space_;

      state_encoder(DiscreteSpace state_space) : boost::static_visitor<std::vector<number>>()
      {
        this->state_space_ = state_space;
      }

      std::vector<number> operator()(number s) const
      {
        return {s};
      }

      std::vector<number> operator()(const std::string &s_str) const
      {
        std::vector<number> st_ptr;
        if (s_str == "*")
        {
          for (state s = 0; s < this->state_space_.getNumElements(); ++s)
          {
            st_ptr.push_back(s);
          }
        }
        else
        {
          st_ptr.push_back(this->state_space_.getElementIndex(s_str));
        }

        return st_ptr;
      }
    };

    //! \struct discrete_space_encoder
    //! \brief encodes the input into a discrete space class
    struct discrete_space_encoder : boost::static_visitor<DiscreteSpace>
    {
      DiscreteSpace operator()(number ag) const
      {
        return DiscreteSpace(ag);
      }

      DiscreteSpace operator()(const std::vector<std::string> &ags) const
      {
        return DiscreteSpace(ags);
      }
    };

    //! \struct multi_discrete_space_encoder
    //! \brief encodes the input into a multi discrete space class
    struct multi_discrete_space_encoder : boost::static_visitor<MultiDiscreteSpace>
    {
      MultiDiscreteSpace operator()(const std::vector<number> &dim_spaces) const
      {
        MultiDiscreteSpace act_space(dim_spaces);
        return act_space;
      }

      MultiDiscreteSpace operator()(const std::vector<std::vector<std::string>> &names) const
      {
        MultiDiscreteSpace act_space(names);
        return act_space;
      }
    };

    //! \struct state_transition_encoder
    //! \brief used to encode state transition dynamics (i.e. StateDynamics class)
    struct state_transition_encoder : boost::static_visitor<>
    {
      std::vector<Matrix> t_model;

      DiscreteSpace st_space_;
      DiscreteSpace ag_space_;
      MultiDiscreteSpace act_space_;

      state_transition_encoder(DiscreteSpace st_space, DiscreteSpace ag_space, MultiDiscreteSpace act_space) : boost::static_visitor<>(), st_space_(st_space), ag_space_(ag_space), act_space_(act_space)
      {
        for (number a = 0; a < this->act_space_.getNumJElements(); ++a)
        {
          auto t = Matrix(this->st_space_.getNumElements(), this->st_space_.getNumElements());

          for (number x = 0; x < this->st_space_.getNumElements(); ++x)
          {
            for (number y = 0; y < this->st_space_.getNumElements(); ++y)
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
        std::vector<number> ja = ja_encoder.encode(t3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumElements(), this->st_space_.getNumElements());
        Matrix prob = boost::apply_visitor(m_encoder, t3.transitions);

        for (number a : ja)
        {
          this->t_model[a] = prob;
        }
      }

      void operator()(const transition_entry_2_t &t2)
      {
        vector_encoder bl_encoder(this->st_space_.getNumElements());
        Vector prob = boost::apply_visitor(bl_encoder, t2.probabilities);

        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<number> ja = ja_encoder.encode(t2.jaction);

        state_encoder x_encoder(this->st_space_);
        auto x_space = boost::apply_visitor(x_encoder, t2.current_state);

        for (number a : ja)
        {
          for (number x : x_space)
          {
            for (number y = 0; y < this->st_space_.getNumElements(); ++y)
            {
              this->t_model[a](x, y) = prob[y];
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
        std::vector<number> ja = ja_encoder.encode(t1.jaction);
        double prob = t1.probability;
        for (number a : ja)
        {
          for (number x : x_space)
          {
            for (number y : y_space)
            {
              this->t_model[a](x, y) = prob;
            }
          }
        }
      }
    };

    class state_dynamics_encoder
    {
    protected:
      DiscreteSpace state_space_;
      DiscreteSpace agent_space_;
      MultiDiscreteSpace action_space_;

    public:
      state_dynamics_encoder(DiscreteSpace state_space, DiscreteSpace agent_space, MultiDiscreteSpace action_space) : state_space_(state_space), agent_space_(agent_space), action_space_(action_space) {}

      StateDynamics encode(const transition_t &transits)
      {
        StateDynamics s_dyn(this->action_space_.getNumJElements(), this->state_space_.getNumElements());
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

      DiscreteSpace st_space_;
      DiscreteSpace ag_space_;
      MultiDiscreteSpace act_space_;
      MultiDiscreteSpace obs_space_;

      observation_transition_encoder(DiscreteSpace st_space, DiscreteSpace ag_space, MultiDiscreteSpace act_space, MultiDiscreteSpace obs_space) : boost::static_visitor<>(), st_space_(st_space), ag_space_(ag_space), act_space_(act_space), obs_space_(obs_space)
      {
        for (number a = 0; a < this->act_space_.getNumJElements(); ++a)
        {
          auto o = Matrix(this->st_space_.getNumElements(), this->obs_space_.getNumJElements());

          for (number x = 0; x < this->st_space_.getNumElements(); ++x)
          {
            for (number z = 0; z < this->obs_space_.getNumJElements(); ++z)
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
        std::vector<number> ja = ja_encoder.encode(z3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumElements(), this->obs_space_.getNumJElements());
        Matrix prob = boost::apply_visitor(m_encoder, z3.probabilities);

        for (number a : ja)
        {
          this->o_model[a] = prob;
        }
      }

      void operator()(const observation_entry_2_t &z2)
      {
        vector_encoder bl_encoder(this->obs_space_.getNumJElements());
        Vector prob = boost::apply_visitor(bl_encoder, z2.probabilities);

        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<number> ja = ja_encoder.encode(z2.jaction);

        state_encoder s_encoder(this->st_space_);
        auto s_space = boost::apply_visitor(s_encoder, z2.next_state);

        for (number a : ja)
          for (number z = 0; z < this->obs_space_.getNumJElements(); ++z)
          {
            for (number y : s_space)
            {
              this->o_model[a](y, z) = prob[z];
            }
          }
      }

      void operator()(const observation_entry_1_t &z1)
      {
        joint_element_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<number> ja = ja_encoder.encode(z1.jaction);

        joint_element_encoder jz_encoder(this->obs_space_, this->ag_space_);
        std::vector<number> jz = jz_encoder.encode(z1.next_observation);

        state_encoder s_encoder(this->st_space_);
        auto y_space = boost::apply_visitor(s_encoder, z1.next_state);

        double prob = z1.probability;
        for (number a : ja)
          for (number z : jz)
          {
            for (number y : y_space)
            {
              this->o_model[a](y, z) = prob;
            }
          }
      }
    };

    class obs_dynamics_encoder
    {
    protected:
      DiscreteSpace state_space_;
      DiscreteSpace agent_space_;
      MultiDiscreteSpace action_space_;
      MultiDiscreteSpace obs_space_;

    public:
      obs_dynamics_encoder(DiscreteSpace state_space, DiscreteSpace agent_space, MultiDiscreteSpace action_space, MultiDiscreteSpace obs_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
        this->obs_space_ = obs_space;
      }

      ObservationDynamics encode(const observation_t &observs, StateDynamics st_dyn)
      {
        ObservationDynamics dynamics(this->action_space_.getNumJElements(), this->obs_space_.getNumJElements(), this->state_space_.getNumElements());
        observation_transition_encoder obs_d_encoder(this->state_space_, this->agent_space_, this->action_space_, this->obs_space_);

        for (observation_entry_t const &obs : observs)
        {
          boost::apply_visitor(obs_d_encoder, obs);
        }
        dynamics.setObservations(obs_d_encoder.o_model);

        for (number u = 0; u < this->action_space_.getNumJElements(); ++u)
        {
          for (number x = 0; x < this->state_space_.getNumElements(); ++x)
          {
            for (number y = 0; y < this->state_space_.getNumElements(); ++y)
            {
              for (number z = 0; z < this->obs_space_.getNumJElements(); ++z)
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
      DiscreteSpace state_space_;
      DiscreteSpace ag_space_;
      MultiDiscreteSpace action_space_;
      Reward *rewards_;

      reward_encoder(DiscreteSpace state_space, DiscreteSpace ag_space, MultiDiscreteSpace action_space, Reward *rewards) : boost::static_visitor<>()
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

        std::vector<number> s_ptr = boost::apply_visitor(s_encoder, r1.state);

        for (number s : s_ptr)
        {
          joint_element_encoder ja_encoder(this->action_space_, this->ag_space_);

          std::vector<number> ja = ja_encoder.encode(r1.jaction);
          for (number a : ja)
          {
            this->rewards_->setReward(s, a, r);
          }
        }
      }

      void operator()(const reward_entry_2_t &r2)
      {
        vector_encoder bl_encoder(this->state_space_.getNumElements());
        Vector v = boost::apply_visitor(bl_encoder, r2.rewards);

        joint_element_encoder ja_encoder(this->action_space_, this->ag_space_);
        std::vector<number> ja = ja_encoder.encode(r2.jaction);
        for (number a : ja)
          this->rewards_->setReward(a, v);
      }
    };

    class rewards_encoder
    {
    protected:
      DiscreteSpace state_space_;
      MultiDiscreteSpace action_space_;
      DiscreteSpace agent_space_;

    public:
      rewards_encoder(DiscreteSpace state_space, DiscreteSpace agent_space, MultiDiscreteSpace action_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
      }

      Reward encode(const reward_t &r)
      {
        Reward rewards(this->action_space_.getNumJElements(), this->state_space_.getNumElements());
        reward_encoder r_encoder(this->state_space_, this->agent_space_, this->action_space_, &rewards);
        for (reward_entry_t const &rew : r)
        {
          boost::apply_visitor(r_encoder, rew);
        }
        return rewards;
      }
    };

    struct decpomdp_encoder : boost::static_visitor<sdm::DecPOMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // DecPOMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      sdm::DecPOMDP operator()(dpomdp_t const &ast)
      {

        discrete_space_encoder ds_encoder;
        multi_discrete_space_encoder mds_encoder;

        // Encodes agent space
        DiscreteSpace agent_space = boost::apply_visitor(ds_encoder, ast.agent_param);

        // Encodes state space
        DiscreteSpace state_space = boost::apply_visitor(ds_encoder, ast.state_param);

        // Encodes action space
        MultiDiscreteSpace action_space = boost::apply_visitor(mds_encoder, ast.action_param);

        // Encodes observation space
        MultiDiscreteSpace obs_space = boost::apply_visitor(mds_encoder, ast.observation_param);

        // Encodes the reward function
        rewards_encoder rews_encoder(state_space, agent_space, action_space);
        Reward rew = rews_encoder.encode(ast.reward_spec);

        // Set start probabilities
        vector_encoder bl_encoder(state_space.getNumElements());
        // auto start_distrib = std::make_shared<Vector>(state_space.getNumElements());
        Vector start_distrib = boost::apply_visitor(bl_encoder, ast.start_param);

        // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        StateDynamics state_dyn = state_dyn_enc.encode(ast.transition_spec);

        // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        ObservationDynamics obs_dyn = d_encoder.encode(ast.observation_spec, state_dyn);

        sdm::DecPOMDP parsed_model(state_space, agent_space, action_space, obs_space, state_dyn, obs_dyn, rew, start_distrib);

        parsed_model.setDiscount(ast.discount_param);
        parsed_model.setCriterion(ast.value_param == "reward");

#ifdef DEBUG
        std::cout << "Model Soundness=" << (parsed_model->isSound() ? "yes" : "no") << std::endl;
#ifdef VERBOSE
        std::cout << "Print model" << std::endl;
        std::cout << *parsed_model << std::endl;
#endif
#endif
        return parsed_model;
      }
    };

  } // namespace ast
} // namespace sdm
