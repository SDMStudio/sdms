#include <sdm/parser/encoders/dynamics_encoders.hpp>

namespace sdm
{

    namespace ast
    {

        state_transition_encoder::state_transition_encoder(const std::shared_ptr<DiscreteSpace> &st_space, const std::shared_ptr<DiscreteSpace> &ag_space, const std::shared_ptr<MultiDiscreteSpace> &act_space, const std::shared_ptr<TabularStateDynamics> &state_dynamics)
            : boost::static_visitor<>(),
              state_space_(st_space),
              agent_space_(ag_space),
              action_space_(act_space),
              state_dynamics_(state_dynamics)
        {
        }

        void state_transition_encoder::operator()(const transition_entry_3_t &t3)
        {
            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(t3.jaction);

            matrix_encoder m_encoder(this->state_space_->getNumItems(), this->state_space_->getNumItems());
            std::shared_ptr<MappedMatrix<number, number>> matrix_proba = boost::apply_visitor(m_encoder, t3.transitions);

            for (const std::shared_ptr<Item> &joint_action : joint_actions)
            {
                for (const auto &pair_state_vector : *matrix_proba)
                {
                    for (const auto &pair_next_state_value : pair_state_vector.second)
                    {
                        this->state_dynamics_->setTransitionProbability(std::static_pointer_cast<State>(this->state_space_->getItem(pair_state_vector.first)), std::static_pointer_cast<Action>(joint_action), std::static_pointer_cast<State>(this->state_space_->getItem(pair_next_state_value.first)), pair_next_state_value.second);
                    }
                }
            }
        }

        void state_transition_encoder::operator()(const transition_entry_2_t &t2)
        {
            vector_encoder bl_encoder(this->state_space_->getNumItems());
            std::shared_ptr<MappedVector<number, double>> prob = boost::apply_visitor(bl_encoder, t2.probabilities);

            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(t2.jaction);

            state_encoder x_encoder(this->state_space_);
            std::vector<std::shared_ptr<Item>> x_space = boost::apply_visitor(x_encoder, t2.current_state);

            for (std::shared_ptr<Item> &joint_action : joint_actions)
            {
                for (std::shared_ptr<Item> &x : x_space)
                {
                    for (const auto &pair_next_state_proba : *prob)
                    {
                        this->state_dynamics_->setTransitionProbability(std::static_pointer_cast<State>(x), std::static_pointer_cast<Action>(joint_action), std::static_pointer_cast<State>(this->state_space_->getItem(pair_next_state_proba.first)), pair_next_state_proba.second);
                    }
                }
            }
        }

        void state_transition_encoder::operator()(const transition_entry_1_t &t1)
        {
            state_encoder s_encoder(this->state_space_);
            std::vector<std::shared_ptr<Item>> x_space = boost::apply_visitor(s_encoder, t1.current_state);
            std::vector<std::shared_ptr<Item>> y_space = boost::apply_visitor(s_encoder, t1.next_state);

            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(t1.jaction);

            double prob = t1.probability;

            for (const std::shared_ptr<Item> &joint_action : joint_actions)
            {
                for (const std::shared_ptr<Item> &x : x_space)
                {
                    for (const std::shared_ptr<Item> &y : y_space)
                    {
                        this->state_dynamics_->setTransitionProbability(std::static_pointer_cast<State>(x), std::static_pointer_cast<Action>(joint_action), std::static_pointer_cast<State>(y), prob);
                    }
                }
            }
        }

        state_dynamics_encoder::state_dynamics_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space)
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

        observation_transition_encoder::observation_transition_encoder(const std::shared_ptr<DiscreteSpace> &st_space, const std::shared_ptr<DiscreteSpace> &ag_space, const std::shared_ptr<MultiDiscreteSpace> &act_space, const std::shared_ptr<MultiDiscreteSpace> &obs_space, const std::shared_ptr<TabularObservationDynamics> &dynamics)
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
            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(z3.jaction);

            matrix_encoder m_encoder(this->state_space_->getNumItems(), this->obs_space_->getNumItems());
            std::shared_ptr<MappedMatrix<number, number>> matrix_proba = boost::apply_visitor(m_encoder, z3.probabilities);

            for (const std::shared_ptr<Item> &state : *this->state_space_)
            {
                for (const std::shared_ptr<Item> &joint_action : joint_actions)
                {
                    for (const auto &pair_next_state_vector : *matrix_proba)
                    {
                        for (const auto &pair_obs_value : pair_next_state_vector.second)
                        {
                            this->obs_dynamics_->setObservationProbability(std::static_pointer_cast<State>(state),
                                                                           std::static_pointer_cast<Action>(joint_action),
                                                                           std::static_pointer_cast<State>(this->state_space_->getItem(pair_next_state_vector.first)),
                                                                           std::static_pointer_cast<Observation>(this->obs_space_->getItem(pair_obs_value.first)),
                                                                           pair_obs_value.second);
                        }
                    }
                }
            }
        }

        void observation_transition_encoder::operator()(const observation_entry_2_t &z2)
        {
            vector_encoder bl_encoder(this->obs_space_->getNumItems());
            std::shared_ptr<MappedVector<number, double>> obs_prob = boost::apply_visitor(bl_encoder, z2.probabilities);

            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(z2.jaction);

            state_encoder s_encoder(this->state_space_);
            std::vector<std::shared_ptr<Item>> next_state_space = boost::apply_visitor(s_encoder, z2.next_state);
            for (const std::shared_ptr<Item> &state : *this->state_space_)
            {
                for (const std::shared_ptr<Item> &joint_action : joint_actions)
                {
                    for (const std::shared_ptr<Item> &next_state : next_state_space)
                    {
                        for (const auto &pair_obs_value : *obs_prob)
                        {
                            this->obs_dynamics_->setObservationProbability(std::static_pointer_cast<State>(state),
                                                                        std::static_pointer_cast<Action>(joint_action),
                                                                        std::static_pointer_cast<State>(next_state),
                                                                        std::static_pointer_cast<Observation>(this->obs_space_->getItem(pair_obs_value.first)),
                                                                        pair_obs_value.second);
                        }
                    }
                }
            }
        }


        void observation_transition_encoder::operator()(const observation_entry_1_t &z1)
        {
            joint_item_encode ja_encoder(this->action_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_actions = ja_encoder.encode(z1.jaction);

            joint_item_encode jz_encoder(this->obs_space_, this->agent_space_);
            std::vector<std::shared_ptr<Item>> joint_observations = jz_encoder.encode(z1.next_observation);

            state_encoder s_encoder(this->state_space_);
            std::vector<std::shared_ptr<Item>> next_state_space = boost::apply_visitor(s_encoder, z1.next_state);

            double prob = z1.probability;
            for (const std::shared_ptr<Item> &state : *this->state_space_)
            {
                for (const std::shared_ptr<Item> &joint_action : joint_actions)
                {
                    for (const std::shared_ptr<Item> &next_state : next_state_space)
                    {
                        for (const std::shared_ptr<Item> &joint_observation : joint_observations)
                        {
                            this->obs_dynamics_->setObservationProbability(std::static_pointer_cast<State>(state),
                                                                        std::static_pointer_cast<Action>(joint_action),
                                                                        std::static_pointer_cast<State>(next_state),
                                                                        std::static_pointer_cast<Observation>(joint_observation),
                                                                        prob);
                        }
                    }
                }
            }
        }


        obs_dynamics_encoder::obs_dynamics_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space, const std::shared_ptr<MultiDiscreteSpace> &obs_space)
        {
            this->state_space_ = state_space;
            this->agent_space_ = agent_space;
            this->action_space_ = action_space;
            this->obs_space_ = obs_space;
        }

        std::shared_ptr<TabularObservationDynamics> obs_dynamics_encoder::encode(const observation_t &observs, std::shared_ptr<StateDynamicsInterface> state_dynamics)
        {
            std::shared_ptr<TabularObservationDynamics> dynamics = std::make_shared<TabularObservationDynamics>();
            observation_transition_encoder obs_d_encoder(this->state_space_, this->agent_space_, this->action_space_, this->obs_space_, dynamics);

            for (observation_entry_t const &obs : observs)
            {
                boost::apply_visitor(obs_d_encoder, obs);
            }

            for (const auto &state : *this->state_space_)
            {
                for (const auto &action : *this->action_space_)
                {
                    for (const auto &next_state : *this->state_space_)
                    {
                        for (const auto &observation : *this->obs_space_)
                        {
                            double obs_proba = dynamics->getObservationProbability(std::static_pointer_cast<State>(state),
                                                                                std::static_pointer_cast<Action>(action),
                                                                                std::static_pointer_cast<State>(next_state),
                                                                                std::static_pointer_cast<Observation>(observation));

                            double transition_proba = state_dynamics->getTransitionProbability(std::static_pointer_cast<State>(state),
                                                                                            std::static_pointer_cast<Action>(action),
                                                                                            std::static_pointer_cast<State>(next_state),
                                                                                            0);

                            dynamics->setDynamics(std::static_pointer_cast<State>(state),
                                                std::static_pointer_cast<Action>(action),
                                                std::static_pointer_cast<State>(next_state),
                                                std::static_pointer_cast<Observation>(observation),
                                                transition_proba * obs_proba);
                        }
                    }
                }
            }
            return dynamics;
        }
    } // namespace ast
} // namespace sdm
