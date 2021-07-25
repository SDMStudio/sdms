#include <sdm/utils/value_function/backup/dqn_phomdp_backup.hpp>

#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/core/observation/default_observation.hpp>

namespace sdm
{
    DqnPhomdpBackup::DqnPhomdpBackup()
    {

    }

    DqnPhomdpBackup::DqnPhomdpBackup(
        std::shared_ptr<ExperienceMemoryInterface> experience_memory, 
        std::shared_ptr<DQN> policy_net, 
        std::shared_ptr<DQN> target_net, 
        double discount, 
        number horizon, 
        number batch_size,
        double lr,
        std::shared_ptr<Space> state_space, 
        std::shared_ptr<Space> observation_space,
        std::shared_ptr<Space> action_space
    ) 
        :   experience_memory_(std::static_pointer_cast<DeepExperienceMemory>(experience_memory)), 
            policy_net_(policy_net), 
            target_net_(target_net), 
            discount_(discount), 
            horizon_(horizon), 
            batch_size_(batch_size),
            lr_(lr),
            state_space_(std::static_pointer_cast<DiscreteSpace>(state_space)), 
            observation_space_(std::static_pointer_cast<MultiDiscreteSpace>(observation_space)), 
            action_space_(std::static_pointer_cast<MultiDiscreteSpace>(action_space))
    {
        torch::optim::AdamOptions options;
		options.lr(lr);
        this->optimizer_ = std::make_shared<torch::optim::Adam>((*this->policy_net_)->parameters(), options);

        this->observation_space_1_ = std::static_pointer_cast<DiscreteSpace>(this->observation_space_->get(0));
        this->observation_space_2_ = std::static_pointer_cast<DiscreteSpace>(this->observation_space_->get(1));
    }

    DqnPhomdpBackup::~DqnPhomdpBackup()
    {

    }

    double DqnPhomdpBackup::update()
    {   
        // std::cout << "DqnPhomdpBackup::update()" << std::endl;

        if (this->experience_memory_->size() < this->batch_size_)
        {
			return 0;
		}


        std::vector<sars_transition> transitions = this->experience_memory_->sample(this->batch_size_);

        auto [t_batch, regular_t_batch, s_batch, z1_batch, z2_batch, u_batch, r_batch, next_t_batch, next_s_batch, next_z1_batch, next_z2_batch, next_u_batch] = this->constructBatch(transitions);

        {
            // std::cout << std::endl;

            // std::cout << "t_batch" << std::endl;
            // std::cout << t_batch << std::endl;

            // std::cout << "regular_t_batch" << std::endl;
            // std::cout << regular_t_batch << std::endl;

            // std::cout << "s_batch" << std::endl;
            // std::cout << s_batch << std::endl;

            // std::cout << "z1_batch" << std::endl;
            // std::cout << z1_batch << std::endl;

            // std::cout << "z2_batch" << std::endl;
            // std::cout << z2_batch << std::endl;

            // std::cout << "u_batch" << std::endl;
            // std::cout << u_batch << std::endl;

            // std::cout << "r_batch" << std::endl;
            // std::cout << r_batch << std::endl;

            // std::cout << "next_t_batch" << std::endl;
            // std::cout << next_t_batch << std::endl;

            // std::cout << "next_s_batch" << std::endl;
            // std::cout << next_s_batch << std::endl;

            // std::cout << "next_z1_batch" << std::endl;
            // std::cout << next_z1_batch << std::endl;

            // std::cout << "next_z2_batch" << std::endl;
            // std::cout << next_z2_batch << std::endl;

            // std::cout << "next_u_batch" << std::endl;
            // std::cout << next_u_batch << std::endl;
        }


        torch::Tensor q_values = this->getQValues(t_batch, s_batch, z1_batch, z2_batch, u_batch);

 

        torch::Tensor target_q_values = this->getTargetQValues(next_t_batch, next_s_batch, next_z1_batch, next_z2_batch, next_u_batch, r_batch, regular_t_batch);

        // std::cout << "q_values" << std::endl;
        // std::cout << q_values << std::endl;

        // std::cout << "target_q_values" << std::endl;
        // std::cout << target_q_values << std::endl;

        torch::Tensor loss = at::smooth_l1_loss(q_values, target_q_values);

        this->optimizer_->zero_grad();

        loss.backward();

        this->optimizer_->step();

        return loss.item<double>();
    }

    std::shared_ptr<Action> DqnPhomdpBackup::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "DqnPhomdpBackup::getGreedyAction()" << std::endl;
        // std::cout << "t " << t << std::endl;

        // 
        if (t == this->horizon_)
        {
            return nullptr;
        }

        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(state)->first;

        // std::cout << *s << std::endl;
        
        auto jhistories = s->getJointHistories();

        // Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
        // Belief
        torch::Tensor private_hierarchical_occupancy_state = torch::zeros(this->state_space_->getNumItems() * this->observation_space_1_->getNumItems());

        // std::cout << private_hierarchical_occupancy_state << std::endl;

        if (t != 0)
        {
            for (const auto& jhistory : jhistories)
            {
                auto belief = s->getBeliefAt(jhistory);
                
                for (number x = 0; x < this->state_space_->getNumItems(); x++)
                {
                    auto z = jhistory->getObservation();
                    auto z1 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0);
                    auto x_z1 = x * this->observation_space_1_->getNumItems() + this->observation_space_1_->getItemIndex(z1);
                    double probability = belief->getProbability(this->state_space_->getItem(x)->toState()) * s->getProbability(jhistory);
                    private_hierarchical_occupancy_state[x_z1] = probability;
                }
            }
        }
        
        // std::cout << private_hierarchical_occupancy_state << std::endl;

        // Create one hot vector for t with correct number of dimensions.
        torch::Tensor one_hot_t = torch::zeros(this->horizon_);
        one_hot_t[t] = 1;

        // std::cout << "///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        // Maps u2 to a1.
        std::unordered_map<std::shared_ptr<Action>, std::shared_ptr<DecisionRule>>  possible_a1s;

        {
            for (const auto & u2: *this->action_space_->get(1))
            {
                std::vector<std::shared_ptr<Item>> inputs;
                std::vector<std::shared_ptr<Item>> outputs;
                for (const auto& jhistory : s->getJointHistories())
                {
                    auto z = jhistory->getObservation();
                    if (z == nullptr)
                        inputs.push_back(NO_OBSERVATION);
                    else
                        inputs.push_back(z);

                    torch::Tensor one_hot_z1 = torch::zeros(this->observation_space_1_->getNumItems());
                    if (t != 0)
                        one_hot_z1[this->observation_space_1_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0))] = 1;

                    torch::Tensor one_hot_z2 = torch::zeros(this->observation_space_2_->getNumItems());
                    if (t != 0)
                        one_hot_z2[this->observation_space_2_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(1))] = 1;

                    torch::Tensor s_z_t = torch::cat({private_hierarchical_occupancy_state, one_hot_z1, one_hot_z2, one_hot_t});

                    torch::Tensor q_values = this->policy_net_->operator()(s_z_t);
                    
                    auto q_values_u1 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();

                    for (const auto & u1: *this->action_space_->get(0))
                    {
                        auto tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                        tmp_u->push_back(u1->toAction());
                        tmp_u->push_back(u2->toAction());

                        auto tmp__u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();

                        auto u = this->action_space_->getItemIndex(tmp__u);

                        q_values_u1->setValueAt(u1->toAction(), q_values.index({u}).item<double>());
                    }

                    auto greedy_u1 = q_values_u1->argmax();
                    outputs.push_back(greedy_u1);
                }
                auto possible_a1 = std::make_shared<DeterministicDecisionRule>(inputs, outputs);
                // std::cout << *possible_a1 << std::endl;
                possible_a1s.emplace(u2->toAction(), possible_a1);
            }
        }

        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 2 /////////////////////////////////" << std::endl;


        std::shared_ptr<DeterministicDecisionRule> a2;
        {
            std::vector<std::shared_ptr<Item>> inputs;
            std::vector<std::shared_ptr<Item>> outputs;
            //
            std::shared_ptr<Observation> z2;
            if (t == 0)
                z2 = NO_OBSERVATION;
            else
                z2 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(state)->second)->at(1);
            
            inputs.push_back(z2);
            torch::Tensor one_hot_z2 = torch::zeros(this->observation_space_2_->getNumItems());
            if (t != 0)
                one_hot_z2[this->observation_space_2_->getItemIndex(z2)] = 1;

            auto q_values_u2 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();

            for (const auto & u2: *this->action_space_->get(1))
            {
                q_values_u2->setValueAt(u2->toAction(), 0);
                for (const auto& jhistory : s->getJointHistories())
                {
                    auto z = jhistory->getObservation();
                    if (z == nullptr)
                        z = NO_OBSERVATION;
                    torch::Tensor one_hot_z1 = torch::zeros(this->observation_space_1_->getNumItems());
                    if (t != 0)
                        one_hot_z1[this->observation_space_1_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0))] = 1;

                    torch::Tensor s_z_t = torch::cat({private_hierarchical_occupancy_state, one_hot_z1, one_hot_z2, one_hot_t});

                    torch::Tensor q_values = this->policy_net_->operator()(s_z_t);
                    
                    auto tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    auto a1 = possible_a1s.at(u2->toAction());
                    tmp_u->push_back(a1->act(z));
                    tmp_u->push_back(u2->toAction());
                    auto tmp__u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();

                    auto u = this->action_space_->getItemIndex(tmp__u);
                    q_values_u2->addValueAt(u2->toAction(), q_values.index({u}).item<double>() * s->getProbability(jhistory));
                }
            }

            auto greedy_u2 = q_values_u2->argmax();
            outputs.push_back(greedy_u2);
            a2 = std::make_shared<DeterministicDecisionRule>(inputs, outputs);
        }

        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        std::shared_ptr<DeterministicDecisionRule> a1;
        {
            std::vector<std::shared_ptr<Item>> inputs;
            std::vector<std::shared_ptr<Item>> outputs;
            
            for (const auto& jhistory : s->getJointHistories())
            {
                auto z = jhistory->getObservation();
                if (z != nullptr)
                {
                    inputs.push_back(z);
                }
                else
                {
                    inputs.push_back(NO_OBSERVATION);
                }

                torch::Tensor one_hot_z1 = torch::zeros(this->observation_space_1_->getNumItems());
                if (t != 0)
                    one_hot_z1[this->observation_space_1_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0))] = 1;

                std::shared_ptr<Observation> z2;
                if (t == 0)
                    z2 = NO_OBSERVATION;
                else
                    z2 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(1);
                torch::Tensor one_hot_z2 = torch::zeros(this->observation_space_2_->getNumItems());
                if (t != 0)
                    one_hot_z2[this->observation_space_2_->getItemIndex(z2)] = 1;

                torch::Tensor s_z_t = torch::cat({private_hierarchical_occupancy_state, one_hot_z1, one_hot_z2, one_hot_t});

                torch::Tensor q_values = this->policy_net_->operator()(s_z_t);

                auto q_values_u1 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
                for (const auto & u1: *this->action_space_->get(0))
                {
                    auto tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    tmp_u->push_back(u1->toAction());
                    tmp_u->push_back(a2->act(z2));

                    auto tmp__u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();

                    auto u = this->action_space_->getItemIndex(tmp__u);
                    q_values_u1->setValueAt(u1->toAction(), q_values.index({u}).item<double>());
                }

                auto greedy_u1 = q_values_u1->argmax();
                outputs.push_back(greedy_u1);
            }
            
            a1 = std::make_shared<DeterministicDecisionRule>(inputs, outputs);
        }
      
        // std::cout << "///////////////////////////////////////////// CREATE JOINT DECISION RULE /////////////////////////////////" << std::endl;

		std::vector<std::shared_ptr<DeterministicDecisionRule>> a;
        a.push_back(a1);
        a.push_back(a2);

        return std::make_shared<JointDeterministicDecisionRule>(a);

        // // Get u.
        // auto u = torch::argmax(this->policy_net_->operator()(s_z_t)).item<int>();
        // // Get the real u from the action space.
        // std::cout << *this->action_space_->getItem(u) << std::endl;
        // return this->action_space_->getItem(u)->toAction();

        return nullptr;

    }

    double DqnPhomdpBackup::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        // return this->target_net_->getQValuesAt(state, t)->max();
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> DqnPhomdpBackup::constructBatch(std::vector<sars_transition> transitions)
    {
        // std::cout << "DqnPhomdpBackup::constructBatch()" << std::endl;

        std::vector<torch::Tensor> t_batch_vector;
        std::vector<long> regular_t_batch_vector;
        std::vector<torch::Tensor> s_batch_vector;
        std::vector<torch::Tensor> z1_batch_vector;
        std::vector<torch::Tensor> z2_batch_vector;
        std::vector<long> u_batch_vector;
        std::vector<long> r_batch_vector;
        std::vector<torch::Tensor> next_t_batch_vector;
        std::vector<torch::Tensor> next_s_batch_vector;
        std::vector<torch::Tensor> next_z1_batch_vector;
        std::vector<torch::Tensor> next_z2_batch_vector;
        std::vector<long> next_u_batch_vector;

        for (sars_transition transition: transitions)
        {
            auto [t, s_z, a, r, next_s_next_z, next_a] = transition;
            auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(s_z)->first;
            auto z = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(s_z)->second;
            auto u = this->applyDecisionRule(z, a, t);
            auto next_s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(next_s_next_z)->first;
            auto next_z = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(next_s_next_z)->second;
            auto next_u = this->applyDecisionRule(next_z, next_a, t+1);

            torch::Tensor one_hot_t = torch::zeros(this->horizon_);
            one_hot_t[t] = 1;
            t_batch_vector.push_back(one_hot_t);
            regular_t_batch_vector.push_back(t);

            torch::Tensor private_hierarchical_occupancy_state = torch::zeros(this->state_space_->getNumItems() * this->observation_space_1_->getNumItems());
            auto jhistories = s->getJointHistories();
            if (t != 0)
            {
                for (const auto& jhistory : jhistories)
                {
                    auto belief = s->getBeliefAt(jhistory);
                    
                    for (number x = 0; x < this->state_space_->getNumItems(); x++)
                    {
                        auto z = jhistory->getObservation();
                        auto z1 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0);
                        auto x_z1 = x * this->observation_space_1_->getNumItems() + this->observation_space_1_->getItemIndex(z1);
                        double probability = belief->getProbability(this->state_space_->getItem(x)->toState()) * s->getProbability(jhistory);
                        private_hierarchical_occupancy_state[x_z1] = probability;
                    }
                }
            }
            s_batch_vector.push_back(private_hierarchical_occupancy_state);

            torch::Tensor one_hot_z1 = torch::zeros(this->observation_space_1_->getNumItems());
            torch::Tensor one_hot_z2 = torch::zeros(this->observation_space_2_->getNumItems());
            if (t != 0)
            {
                one_hot_z1[this->observation_space_1_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(0))] = 1;
                one_hot_z2[this->observation_space_2_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(1))] = 1;
            }
            z1_batch_vector.push_back(one_hot_z1);
            z2_batch_vector.push_back(one_hot_z2);

            u_batch_vector.push_back(this->action_space_->getItemIndex(u));

            r_batch_vector.push_back(r);

            torch::Tensor one_hot_next_t = torch::zeros(this->horizon_);
            if (t + 1 != this->horizon_)
            {
                one_hot_next_t[t+1] = 1;
            }
            next_t_batch_vector.push_back(one_hot_next_t);


            torch::Tensor next_private_hierarchical_occupancy_state = torch::zeros(this->state_space_->getNumItems() * this->observation_space_1_->getNumItems());
            auto next_jhistories = next_s->getJointHistories();
            for (const auto& next_jhistory : next_jhistories)
            {
                auto next_belief = next_s->getBeliefAt(next_jhistory);
                
                for (number next_x = 0; next_x < this->state_space_->getNumItems(); next_x++)
                {
                    auto next_z = next_jhistory->getObservation();
                    auto next_z1 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_z)->at(0);
                    auto next_x_next_z1 = next_x * this->observation_space_1_->getNumItems() + this->observation_space_1_->getItemIndex(next_z1);
                    double probability = next_belief->getProbability(this->state_space_->getItem(next_x)->toState()) * next_s->getProbability(next_jhistory);
                    next_private_hierarchical_occupancy_state[next_x_next_z1] = probability;
                }
            }
            next_s_batch_vector.push_back(next_private_hierarchical_occupancy_state);


            torch::Tensor one_hot_next_z1 = torch::zeros(this->observation_space_1_->getNumItems());
            torch::Tensor one_hot_next_z2 = torch::zeros(this->observation_space_2_->getNumItems());
            one_hot_next_z1[this->observation_space_1_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_z)->at(0))] = 1;
            one_hot_next_z2[this->observation_space_2_->getItemIndex(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_z)->at(1))] = 1;
            next_z1_batch_vector.push_back(one_hot_next_z1);
            next_z2_batch_vector.push_back(one_hot_next_z2);

            next_u_batch_vector.push_back(this->action_space_->getItemIndex(next_u));

        }

        torch::Tensor t_batch = torch::cat(t_batch_vector);
        t_batch = t_batch.reshape({this->batch_size_, -1});

        torch::Tensor regular_t_batch = torch::tensor(regular_t_batch_vector);

        torch::Tensor s_batch = torch::cat(s_batch_vector);
        s_batch = s_batch.reshape({this->batch_size_, -1});

        torch::Tensor z1_batch = torch::cat(z1_batch_vector);
        z1_batch = z1_batch.reshape({this->batch_size_, -1});

        torch::Tensor z2_batch = torch::cat(z2_batch_vector);
        z2_batch = z2_batch.reshape({this->batch_size_, -1});

        torch::Tensor u_batch = torch::tensor(u_batch_vector);
        u_batch = u_batch.reshape({this->batch_size_, -1});

        torch::Tensor r_batch = torch::tensor(r_batch_vector);

        torch::Tensor next_t_batch = torch::cat(next_t_batch_vector);
        next_t_batch = next_t_batch.reshape({this->batch_size_, -1});
        
        torch::Tensor next_s_batch = torch::cat(next_s_batch_vector);
        next_s_batch = next_s_batch.reshape({this->batch_size_, -1});

        torch::Tensor next_z1_batch = torch::cat(next_z1_batch_vector);
        next_z1_batch = next_z1_batch.reshape({this->batch_size_, -1});

        torch::Tensor next_z2_batch = torch::cat(next_z2_batch_vector);
        next_z2_batch = next_z2_batch.reshape({this->batch_size_, -1});

        torch::Tensor next_u_batch = torch::tensor(next_u_batch_vector);
        next_u_batch = next_u_batch.reshape({this->batch_size_, -1});

        return std::make_tuple(t_batch, regular_t_batch, s_batch, z1_batch, z2_batch, u_batch, r_batch, next_t_batch, next_s_batch, next_z1_batch, next_z2_batch, next_u_batch);
    }

    torch::Tensor DqnPhomdpBackup::getQValues(torch::Tensor t_batch, torch::Tensor s_batch, torch::Tensor z1_batch, torch::Tensor z2_batch, torch::Tensor u_batch)
    {
        torch::Tensor s_z1_z2_t_batch = torch::cat({s_batch, z1_batch, z2_batch, t_batch}, 1);

		return this->policy_net_->operator()(s_z1_z2_t_batch).gather(-1, u_batch);
    }

    torch::Tensor DqnPhomdpBackup::getTargetQValues(torch::Tensor next_t_batch, torch::Tensor next_s_batch, torch::Tensor next_z1_batch, torch::Tensor next_z2_batch, torch::Tensor next_u_batch, torch::Tensor r_batch, torch::Tensor regular_t_batch)
    {
        // Let PyTorch know that we don't need to keep track of the gradient in this context.
		torch::NoGradGuard no_grad;
        torch::Tensor next_s_next_z_next_t_batch = torch::cat({next_s_batch, next_z1_batch, next_z2_batch, next_t_batch}, 1);
        torch::Tensor next_state_values = this->target_net_->operator()(next_s_next_z_next_t_batch).gather(-1, next_u_batch).squeeze();
        torch::Tensor zeros = torch::zeros(this->batch_size_);
        torch::Tensor correct_next_state_values = torch::where(regular_t_batch < this->horizon_ - 1, next_state_values, zeros);
        torch::Tensor target_q_values = (correct_next_state_values * this->discount_) + r_batch;
        return target_q_values.unsqueeze(1);
    }

    std::shared_ptr<Action> DqnPhomdpBackup::applyDecisionRule(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &decision_rule, number t) const
    {

        std::shared_ptr<Joint<std::shared_ptr<Observation>>> joint_hierarchical_observations = std::make_shared<Joint<std::shared_ptr<Observation>>>();
        if (observation == nullptr)
        {
            joint_hierarchical_observations->push_back(NO_OBSERVATION);
            joint_hierarchical_observations->push_back(NO_OBSERVATION);
        }
        else
        {

            std::shared_ptr<Observation> observation_1 = observation;
            std::shared_ptr<Observation> observation_2 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(1);
            joint_hierarchical_observations->push_back(observation_1);
            joint_hierarchical_observations->push_back(observation_2);
        }

       // Get the selected action
        auto action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_hierarchical_observations));

        // Get the adress of the action object from the space of available action object.
        return this->action_space_->getItemAddress(*action->toJoint<Item>())->toAction();
    }
}