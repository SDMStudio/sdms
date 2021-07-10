#include <sdm/world/sampled_belief_mdp.hpp>


namespace sdm
{

    template <class TBelief>
    SampledBaseBeliefMDP<TBelief>::SampledBaseBeliefMDP()
    {
    }

    // SampledBaseBeliefMDP<TBelief>::SampledBaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, const Belief &initial_belief)
    //     : SolvableByMDP(pomdp)
    // {
    //     this->initial_state_ = std::make_shared<BeliefStateGraph>(initial_belief);
    //     this->current_state_ = std::make_shared<BeliefStateGraph>(*std::static_pointer_cast<TBelief>(this->initial_state_));
    // }

    template <class TBelief>
    SampledBaseBeliefMDP<TBelief>::SampledBaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, int batch_size) : BaseBeliefMDP<TBelief>(pomdp), batch_size_(batch_size)
    {
        this->initial_state_ = std::make_shared<TBelief>();

        auto start_distribution = pomdp->getStartDistribution();

        for (int b = 0; b < this->batch_size_; b++)
        {
            // std::cout << *this->initial_state_ << std::endl;
            auto x = start_distribution->sample();
            // std::cout << *x << std::endl;
            this->initial_x_vector_.push_back(x);
            this->initial_state_->toBelief()->addProbability(x, 1. / this->batch_size_);
        }

        // std::cout << *this->initial_state_ << std::endl;
        // std::cout << this->batch_size_ << std::endl;

        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, double> SampledBaseBeliefMDP<TBelief>::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        std::shared_ptr<TBelief> next_belief = std::make_shared<TBelief>();

        for (int b = 1; b < this->batch_size_; b++)
        {
            auto x = this->current_x_vector_[b];
            std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->setInternalState(x);
            auto feedback = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
            auto y = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->getInternalState();
            this->current_x_vector_[b] = y;
            this->rewards_vector.push_back(std::get<1>(feedback)[0]);
            next_belief->addProbability(y, 1. / this->batch_size_);
        }

        // Compute the coefficient of normalization (eta)
        double eta = next_belief->norm_1();
        if (eta > 0)
        {
            // Normalize the belief
            for (const auto &state : next_belief->getStates())
            {
                next_belief->setProbability(state->toState(), next_belief->getProbability(state->toState()) / eta);
            }
        }
        return {next_belief, eta};
    }

    template <class TBelief>
    std::shared_ptr<Observation> SampledBaseBeliefMDP<TBelief>::reset()
    {
        this->current_x_vector_ = this->initial_x_vector_;
        return BaseBeliefMDP<TBelief>::reset();
    }

    template <class TBelief>
    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> SampledBaseBeliefMDP<TBelief>::step(std::shared_ptr<Action> action)
    {
        // auto [next_obs, rewards, done] = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
        auto x = this->current_x_vector_[0];
        std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->setInternalState(x);
        auto feedback = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
        auto y = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->getInternalState();
        this->current_x_vector_[0] = y;
        this->rewards_vector.clear();
        this->rewards_vector.push_back(std::get<1>(feedback)[0]);
        auto next_obs = std::get<0>(feedback);
        this->current_state_ = this->nextBelief(this->current_state_, action, next_obs, this->step_);
        this->b_star_ = rand() % this->batch_size_;
        return std::make_tuple(this->current_state_, std::vector<double>{this->rewards_vector[this->b_star_]}, std::get<2>(feedback));
    }

} // namespace sdm
