/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/core/dpomdp.hpp>
#include <sdm/core/__state__.hpp>
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

    struct action_encoder : boost::static_visitor<action>
    {
      __action__ act_space_;
      agent ag;

      action_encoder(__action__ act_space, agent ag) : boost::static_visitor<action>()
      {
        this->ag = ag;
        this->act_space_ = act_space;
      }

      action operator()(action a) const
      {
        return a;
      }

      action operator()(const std::string &a_str) const
      {
        return this->act_space_.getActionIndex(this->ag, a_str);
      }
    };

    class joint_action_encoder
    {
    protected:
      __action__ act_space_;
      __agent__ ag_space_;

    public:
      joint_action_encoder(__action__ act_space, __agent__ ag_space)
      {
        this->ag_space_ = ag_space;
        this->act_space_ = act_space;
      }

      std::vector<action> encode(const std::vector<identifier_t> &as) const
      {
        std::vector<action> a_vec;
        if (as.size() == 1)
        {
          for (action a = 0; a < this->act_space_.getNumActions(); ++a)
          {
            a_vec.push_back(a);
          }
        }
        else
        {
          std::vector<action> ja;
          for (agent ag = 0; ag < this->ag_space_.getNumAgents(); ++ag)
          {
            action_encoder a_encoder(this->act_space_, ag);
            action a_ = boost::apply_visitor(a_encoder, as[ag]);
            ja.push_back(a_);
          }
          a_vec.push_back(joint_action::getJointItemIdx(ja));
        }
        return a_vec;
      }
    };

    struct observation_encoder : boost::static_visitor<observation>
    {
      __observation__ obs_space_;
      agent ag;

      observation_encoder(__observation__ obs_space, agent ag) : boost::static_visitor<observation>()
      {
        this->ag = ag;
        this->obs_space_ = obs_space;
      }

      observation operator()(observation o) const
      {
        return o;
      }

      observation operator()(const std::string &o_str) const
      {
        return this->obs_space_.getObservationIndex(this->ag, o_str);
      }
    };

    class joint_observation_encoder
    {
    protected:
      __observation__ obs_space_;
      __agent__ ag_space_;

    public:
      joint_observation_encoder(__observation__ obs_space, __agent__ ag_space)
      {
        this->ag_space_ = ag_space;
        this->obs_space_ = obs_space;
      }

      std::vector<observation> encode(const std::vector<identifier_t> &os) const
      {
        std::vector<observation> o_vec;
        if (os.size() == 1)
        {
          for (observation o = 0; o < this->obs_space_.getNumObservations(); ++o)
          {
            o_vec.push_back(o);
          }
        }
        else
        {
          std::vector<observation> jo;
          for (agent ag = 0; ag < this->ag_space_.getNumAgents(); ++ag)
          {
            observation_encoder o_encoder(this->obs_space_, ag);
            jo.push_back(boost::apply_visitor(o_encoder, os[ag]));
          }

          o_vec.push_back(joint_observation::getJointItemIdx(jo));
        }

        return o_vec;
      }
    };

    struct state_encoder : boost::static_visitor<std::vector<state>>
    {

      __state__ state_space_;

      state_encoder(__state__ state_space) : boost::static_visitor<std::vector<state>>()
      {
        this->state_space_ = state_space;
      }

      std::vector<state> operator()(state s) const
      {
        std::vector<state> st_ptr;
        st_ptr.push_back(s);
        return st_ptr;
      }

      std::vector<state> operator()(const std::string &s_str) const
      {
        std::vector<state> st_ptr;
        if (s_str == "*")
        {
          for (state s = 0; s < this->state_space_.getNumStates(); ++s)
          {
            st_ptr.push_back(s);
          }
        }
        else
        {
          st_ptr.push_back(this->state_space_.getStateIndex(s_str));
        }

        return st_ptr;
      }
    };

    struct agent_encoder : boost::static_visitor<__agent__>
    {
      __agent__ operator()(agent ag) const
      {
        return __agent__(ag);
      }

      __agent__ operator()(const std::vector<std::string> &ags) const
      {
        return __agent__(ags);
      }
    };

    struct state_space_encoder : boost::static_visitor<__state__>
    {
      __state__ operator()(state s) const
      {
        return __state__(s);
      }

      __state__ operator()(const std::vector<std::string> &ss) const
      {
        return __state__(ss);
      }
    };

    struct action_space_encoder : boost::static_visitor<__action__>
    {
      __action__ operator()(const std::vector<action> &actions) const
      {
        __action__ act_space;
        act_space.setNumActions(actions);
        return act_space;
      }

      __action__ operator()(const std::vector<std::vector<std::string>> &actions_str) const
      {
        __action__ act_space;
        act_space.setNumActions(actions_str);
        return act_space;
      }
    };

    struct observation_space_encoder : boost::static_visitor<__observation__>
    {
      __observation__ operator()(const std::vector<observation> &observations) const
      {
        __observation__ obs_space;
        obs_space.setNumObservations(observations);
        return obs_space;
      }

      __observation__ operator()(const std::vector<std::vector<std::string>> &observations_str) const
      {
        __observation__ obs_space;
        obs_space.setNumObservations(observations_str);
        return obs_space;
      }
    };

    struct dynamic_encoder : boost::static_visitor<>
    {
      std::vector<Matrix> t_model, o_model;

      __state__ st_space_;
      __agent__ ag_space_;
      __action__ act_space_;
      __observation__ obs_space_;

      dynamic_encoder(__state__ st_space, __agent__ ag_space, __action__ act_space, __observation__ obs_space) : boost::static_visitor<>(), st_space_(st_space), ag_space_(ag_space), act_space_(act_space), obs_space_(obs_space)
      {
        for (action a = 0; a < this->act_space_.getNumActions(); ++a)
        {
          auto t = Matrix(this->st_space_.getNumStates(), this->st_space_.getNumStates());
          auto o = Matrix(this->st_space_.getNumStates(), this->obs_space_.getNumObservations());

          for (state x = 0; x < this->st_space_.getNumStates(); ++x)
          {
            for (state y = 0; y < this->st_space_.getNumStates(); ++y)
            {
              t(x, y) = 0.0;
            }

            for (observation z = 0; z < this->obs_space_.getNumObservations(); ++z)
            {
              o(x, z) = 0.0;
            }
          }

          this->t_model.push_back(t);
          this->o_model.push_back(o);
        }
      }

      void operator()(const observation_entry_3_t &z3)
      {
        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(z3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumStates(), this->obs_space_.getNumObservations());
        Matrix prob = boost::apply_visitor(m_encoder, z3.probabilities);

        for (action a : ja)
        {
          this->o_model[a] = prob;
        }
      }

      void operator()(const transition_entry_3_t &t3)
      {

        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(t3.jaction);

        matrix_encoder m_encoder(this->st_space_.getNumStates(), this->st_space_.getNumStates());
        Matrix prob = boost::apply_visitor(m_encoder, t3.transitions);

        for (action a : ja)
        {
          this->t_model[a] = prob;
        }
      }

      void operator()(const observation_entry_2_t &z2)
      {
        vector_encoder bl_encoder(this->obs_space_.getNumObservations());
        Vector prob = boost::apply_visitor(bl_encoder, z2.probabilities);

        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(z2.jaction);

        state_encoder s_encoder(this->st_space_);
        auto s_space = boost::apply_visitor(s_encoder, z2.next_state);

        for (action a : ja)
          for (observation z = 0; z < this->obs_space_.getNumObservations(); ++z)
          {
            for (state y : s_space)
            {
              this->o_model[a](y, z) = prob[z];
            }
          }
      }

      void operator()(const transition_entry_2_t &t2)
      {
        vector_encoder bl_encoder(this->st_space_.getNumStates());
        Vector prob = boost::apply_visitor(bl_encoder, t2.probabilities);

        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(t2.jaction);

        state_encoder x_encoder(this->st_space_);
        auto x_space = boost::apply_visitor(x_encoder, t2.current_state);

        for (action a : ja)
        {
          for (state x : x_space)
          {
            for (state y = 0; y < this->st_space_.getNumStates(); ++y)
            {
              this->t_model[a](x, y) = prob[y];
            }
          }
        }
      }

      void operator()(const observation_entry_1_t &z1)
      {
        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(z1.jaction);

        joint_observation_encoder jz_encoder(this->obs_space_, this->ag_space_);
        std::vector<observation> jz = jz_encoder.encode(z1.next_observation);

        state_encoder s_encoder(this->st_space_);
        auto y_space = boost::apply_visitor(s_encoder, z1.next_state);

        double prob = z1.probability;
        for (action a : ja)
          for (observation z : jz)
          {
            for (state y : y_space)
            {
              this->o_model[a](y, z) = prob;
            }
          }
      }

      void operator()(const transition_entry_1_t &t1)
      {
        state_encoder s_encoder(this->st_space_);
        auto y_space = boost::apply_visitor(s_encoder, t1.next_state);
        auto x_space = boost::apply_visitor(s_encoder, t1.current_state);
        joint_action_encoder ja_encoder(this->act_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(t1.jaction);
        double prob = t1.probability;
        for (action a : ja)
        {
          for (state x : x_space)
          {
            for (state y : y_space)
            {
              this->t_model[a](x, y) = prob;
            }
          }
        }
      }
    };

    class dynamics_encoder
    {
    protected:
      __state__ state_space_;
      __agent__ agent_space_;
      __action__ action_space_;
      __observation__ obs_space_;

    public:
      dynamics_encoder(__state__ state_space, __agent__ agent_space, __action__ action_space, __observation__ obs_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
        this->obs_space_ = obs_space;
      }

      __dynamics__ encode(const transition_t &transits, const observation_t &observs)
      {
        __dynamics__ dynamics;
        dynamics.initDynamics(this->action_space_.getNumActions(), this->obs_space_.getNumObservations(), this->state_space_.getNumStates());

        dynamic_encoder d_encoder(this->state_space_, this->agent_space_, this->action_space_, this->obs_space_);
        for (transition_entry_t const &tr : transits)
        {
          boost::apply_visitor(d_encoder, tr);
        }
        dynamics.setTransitions(d_encoder.t_model);

        for (observation_entry_t const &obs : observs)
        {
          boost::apply_visitor(d_encoder, obs);
        }
        dynamics.setObservations(d_encoder.o_model);

        for (action u = 0; u < this->action_space_.getNumActions(); ++u)
        {
          for (state x = 0; x < this->state_space_.getNumStates(); ++x)
          {
            for (state y = 0; y < this->state_space_.getNumStates(); ++y)
            {
              for (observation z = 0; z < this->obs_space_.getNumObservations(); ++z)
              {
                dynamics.setDynamics(x, u, z, y, d_encoder.t_model[u](x, y) * d_encoder.o_model[u](y, z));
              }
            }
          }
        }
        return dynamics;
      }
    };

    struct reward_encoder : boost::static_visitor<>
    {
      __state__ state_space_;
      __agent__ ag_space_;
      __action__ action_space_;
      __reward__ *rewards_;

      reward_encoder(__state__ state_space, __agent__ ag_space, __action__ action_space, __reward__ *rewards) : boost::static_visitor<>()
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

        std::vector<state> s_ptr = boost::apply_visitor(s_encoder, r1.state);

        for (state s : s_ptr)
        {
          joint_action_encoder ja_encoder(this->action_space_, this->ag_space_);

          std::vector<action> ja = ja_encoder.encode(r1.jaction);
          for (action a : ja)
          {
            this->rewards_->setReward(s, a, r);
          }
        }
      }

      void operator()(const reward_entry_2_t &r2)
      {
        vector_encoder bl_encoder(this->state_space_.getNumStates());
        Vector v = boost::apply_visitor(bl_encoder, r2.rewards);

        joint_action_encoder ja_encoder(this->action_space_, this->ag_space_);
        std::vector<action> ja = ja_encoder.encode(r2.jaction);
        for (action a : ja)
          this->rewards_->setReward(a, v);
      }
    };

    class rewards_encoder
    {
    protected:
      __state__ state_space_;
      __action__ action_space_;
      __agent__ agent_space_;

    public:
      rewards_encoder(__state__ state_space, __agent__ agent_space, __action__ action_space)
      {
        this->state_space_ = state_space;
        this->agent_space_ = agent_space;
        this->action_space_ = action_space;
      }

      __reward__ encode(const reward_t &r)
      {
        __reward__ rewards;
        rewards.initReward(this->action_space_.getNumActions(), this->state_space_.getNumStates());
        reward_encoder r_encoder(this->state_space_, this->agent_space_, this->action_space_, &rewards);
        for (reward_entry_t const &rew : r)
        {
          boost::apply_visitor(r_encoder, rew);
        }
        return rewards;
      }
    };

    struct dpomdp_encoder : boost::static_visitor<sdm::dpomdp>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // dpomdp encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      sdm::dpomdp operator()(dpomdp_t const &ast)
      {

        sdm::dpomdp parsed_model;
        // parsed_model = std::make_shared<sdm::dpomdp>();

        parsed_model.setDiscount(ast.discount_param);
        parsed_model.setCriterion(ast.value_param == "reward");

        // Encodes agent space
        agent_encoder ag_encoder;
        parsed_model.setAgentSpace(boost::apply_visitor(ag_encoder, ast.agent_param));

        // Encodes state space
        state_space_encoder st_encoder;
        parsed_model.setStateSpace(boost::apply_visitor(st_encoder, ast.state_param));

        // Encodes action space
        action_space_encoder a_space_encoder;
        parsed_model.setActionSpace(boost::apply_visitor(a_space_encoder, ast.action_param));

        // Encodes observation space
        observation_space_encoder o_encoder;
        parsed_model.setObservationSpace(boost::apply_visitor(o_encoder, ast.observation_param));

        // Encodes the reward function
        rewards_encoder rews_encoder(parsed_model.getStateSpace(), parsed_model.getAgentSpace(), parsed_model.getActionSpace());
        parsed_model.setReward(rews_encoder.encode(ast.reward_spec));

        // // Set start probabilities
        auto v = std::make_shared<Vector>(parsed_model.getStateSpace().getNumStates());
        vector_encoder bl_encoder(parsed_model.getStateSpace().getNumStates());
        *v = boost::apply_visitor(bl_encoder, ast.start_param);
        parsed_model.setStart(v);

        // // Encodes the dynamics
        dynamics_encoder d_encoder(parsed_model.getStateSpace(), parsed_model.getAgentSpace(), parsed_model.getActionSpace(), parsed_model.getObservationSpace());
        parsed_model.setDynamics(d_encoder.encode(ast.transition_spec, ast.observation_spec));

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
