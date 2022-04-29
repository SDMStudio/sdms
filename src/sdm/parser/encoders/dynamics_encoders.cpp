#include <sdm/parser/encoders/dynamics_encoders.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/observation/base_observation.hpp>

namespace sdm
{

    namespace ast
    {

        state_transition_encoder::state_transition_encoder(const std::shared_ptr<DiscreteStateSpace> &st_space, const std::shared_ptr<DiscreteItemSpace> &ag_space, const std::shared_ptr<MultiDiscreteActionSpace> &act_space, const std::shared_ptr<TabularStateDynamics> &state_dynamics)
            : boost::static_visitor<>(),
              state_space_(st_space),
              agent_space_(ag_space),
              action_space_(act_space),
              state_dynamics_(state_dynamics)
        {
        }

        void state_transition_encoder::operator()(const transition_entry_3_t &t3)
        {
            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(t3.jaction);

            matrix_encoder m_encoder(this->state_space_->getNumItems(), this->state_space_->getNumItems());
            std::shared_ptr<MappedMatrix<number, number>> matrix_proba = boost::apply_visitor(m_encoder, t3.transitions);


            for (const auto &u : u_space)
            {
                for (const auto &pair_state_vector : *matrix_proba)
                {
                    for (const auto &pair_next_state_value : pair_state_vector.second)
                    {
                        this->state_dynamics_->setTransitionProbability(this->state_space_->getItem(pair_state_vector.first), u, this->state_space_->getItem(pair_next_state_value.first), pair_next_state_value.second);
                    }
                }
            }
        }

        void state_transition_encoder::operator()(const transition_entry_2_t &t2)
        {
            vector_encoder bl_encoder(this->state_space_->getNumItems());
            std::shared_ptr<MappedVector<number, double>> prob = boost::apply_visitor(bl_encoder, t2.probabilities);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(t2.jaction);

            state_encoder<StringState> x_encoder(this->state_space_);
            auto x_space = boost::apply_visitor(x_encoder, t2.current_state);

            for (auto &u : u_space)
            {
                for (auto &x : x_space)
                {
                    for (const auto &pair_next_state_proba : *prob)
                    {
                        this->state_dynamics_->setTransitionProbability(x, u, this->state_space_->getItem(pair_next_state_proba.first), pair_next_state_proba.second);
                    }
                }
            }
        }

        void state_transition_encoder::operator()(const transition_entry_1_t &t1)
        {
            state_encoder<StringState> s_encoder(this->state_space_);
            auto x_space = boost::apply_visitor(s_encoder, t1.current_state);
            auto y_space = boost::apply_visitor(s_encoder, t1.next_state);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(t1.jaction);

            double prob = t1.probability;

            for (const auto &u : u_space)
            {
                for (const auto &x : x_space)
                {
                    for (const auto &y : y_space)
                    {
                        this->state_dynamics_->setTransitionProbability(x, u, y, prob);
                    }
                }
            }
        }

        state_dynamics_encoder::state_dynamics_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space, const std::shared_ptr<DiscreteItemSpace> &agent_space, const std::shared_ptr<MultiDiscreteActionSpace> &action_space)
        {
            this->state_space_ = state_space;
            this->agent_space_ = agent_space;
            this->action_space_ = action_space;
        }

        std::shared_ptr<TabularStateDynamics> state_dynamics_encoder::encode(const transition_t &transits)
        {
            std::shared_ptr<TabularStateDynamics> state_dyn = std::make_shared<TabularStateDynamics>();
            state_transition_encoder state_d_encoder(this->state_space_, this->agent_space_, this->action_space_, state_dyn);
            for (transition_entry_t const &tr : transits)
            {
                boost::apply_visitor(state_d_encoder, tr);
            }
            return state_dyn;
        }

        // ################################################################
        // ############ OBSERVATION DYNAMICS ENCODER ######################
        // ################################################################

        observation_transition_encoder::observation_transition_encoder(const std::shared_ptr<DiscreteStateSpace> &st_space, const std::shared_ptr<DiscreteItemSpace> &ag_space, const std::shared_ptr<MultiDiscreteActionSpace> &act_space, const std::shared_ptr<MultiDiscreteObservationSpace> &obs_space, const std::shared_ptr<TabularObservationDynamicsAS> &dynamics)
            : boost::static_visitor<>(),
              state_space_(st_space),
              agent_space_(ag_space),
              action_space_(act_space),
              obs_space_(obs_space),
              obs_dynamics_(dynamics)
        {
        }

        void observation_transition_encoder::operator()(const observation_entry_3_t &z3)
        {
            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(z3.jaction);

            matrix_encoder m_encoder(this->state_space_->getNumItems(), this->obs_space_->getNumItems());
            std::shared_ptr<MappedMatrix<number, number>> matrix_proba = boost::apply_visitor(m_encoder, z3.probabilities);

            auto end_iter = this->state_space_->end();
            for (auto iter = this->state_space_->begin(); !iter->equal(end_iter); iter = iter->next())
            {
                auto state = iter->getCurrent();
                for (const auto &u : u_space)
                {
                    for (const auto &pair_next_state_vector : *matrix_proba)
                    {
                        for (const auto &pair_obs_value : pair_next_state_vector.second)
                        {
                            this->obs_dynamics_->setObservationProbability(state, u, this->state_space_->getItem(pair_next_state_vector.first), this->obs_space_->getItem(pair_obs_value.first), pair_obs_value.second);
                        }
                    }
                }
            }
        }

        void observation_transition_encoder::operator()(const observation_entry_2_t &z2)
        {
            vector_encoder bl_encoder(this->obs_space_->getNumItems());
            std::shared_ptr<MappedVector<number, double>> obs_prob = boost::apply_visitor(bl_encoder, z2.probabilities);

            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(z2.jaction);

            state_encoder<StringState> s_encoder(this->state_space_);
            auto next_state_space = boost::apply_visitor(s_encoder, z2.next_state);

            auto end_iter = this->state_space_->end();
            for (auto iter = this->state_space_->begin(); !iter->equal(end_iter); iter = iter->next())
            {
                auto state = iter->getCurrent();
                for (const auto &u : u_space)
                {
                    for (const auto &next_state : next_state_space)
                    {
                        for (const auto &pair_obs_value : *obs_prob)
                        {
                            this->obs_dynamics_->setObservationProbability(state, u, next_state, this->obs_space_->getItem(pair_obs_value.first), pair_obs_value.second);
                        }
                    }
                }
            }
        }

        void observation_transition_encoder::operator()(const observation_entry_1_t &z1)
        {
            joint_item_encoder<StringAction> ja_encoder(this->action_space_, this->agent_space_);
            auto u_space = ja_encoder.encode(z1.jaction);

            joint_item_encoder<StringObservation> jz_encoder(this->obs_space_, this->agent_space_);
            auto z_space = jz_encoder.encode(z1.next_observation);

            state_encoder<StringState> s_encoder(this->state_space_);
            auto next_state_space = boost::apply_visitor(s_encoder, z1.next_state);

            double prob = z1.probability;

            auto end_iter = this->state_space_->end();
            for (auto iter = this->state_space_->begin(); !iter->equal(end_iter); iter = iter->next())
            {
                auto state = iter->getCurrent();
                for (const auto &u : u_space)
                {
                    for (const auto &next_state : next_state_space)
                    {
                        for (const auto &joint_observation : z_space)
                        {
                            this->obs_dynamics_->setObservationProbability(state, u, next_state, joint_observation, prob);
                        }
                    }
                }
            }
        }

        obs_dynamics_encoder::obs_dynamics_encoder(const std::shared_ptr<DiscreteStateSpace> &state_space, const std::shared_ptr<DiscreteItemSpace> &agent_space, const std::shared_ptr<MultiDiscreteActionSpace> &action_space, const std::shared_ptr<MultiDiscreteObservationSpace> &obs_space)
        {
            this->state_space_ = state_space;
            this->agent_space_ = agent_space;
            this->action_space_ = action_space;
            this->obs_space_ = obs_space;
        }

        std::shared_ptr<TabularObservationDynamicsAS> obs_dynamics_encoder::encode(const observation_t &observs, std::shared_ptr<StateDynamicsInterface>)
        {
            std::shared_ptr<TabularObservationDynamicsAS> dynamics = std::make_shared<TabularObservationDynamicsAS>();
            observation_transition_encoder obs_d_encoder(this->state_space_, this->agent_space_, this->action_space_, this->obs_space_, dynamics);

            for (observation_entry_t const &obs : observs)
            {
                boost::apply_visitor(obs_d_encoder, obs);
            }
            return dynamics;
        }
    } // namespace ast
} // namespace sdm
