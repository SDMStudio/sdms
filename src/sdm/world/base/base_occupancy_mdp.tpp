#include <sdm/world/base/base_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    OccupancyMDP::OccupancyMDP()
    {
    }

    OccupancyMDP::OccupancyMDP(std::shared_ptr<BasePOMDP> dpomdp, number) : BeliefMDP(dpomdp)
    {
    }

    OccupancyMDP::OccupancyMDP(std::string dpomdp, number hist_length) : OccupancyMDP(std::make_shared<BasePOMDP>(dpomdp), hist_length)
    {
    }

    std::shared_ptr<State> OccupancyMDP::reset()
    {
        // Reset the joint history to initial value
        this->current_history_ = this->initial_history_;

        // Reset the occupancy state
        *this->current_state_ = *this->initial_state_;

        // Reset the underlying DecPOMDP
        this->getUnderlyingProblem()->reset();

        // Return the occupancy (which is the observation in OccupancyMDP formalism)
        return *this->current_state_;
    }

    bool OccupancyMDP::isSerialized() const
    {
        return false;
    }

    std::shared_ptr<State> OccupancyMDP::getInitialState()
    {
        return *this->initial_state_;
    }

    std::shared_ptr<DiscreteSpace> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate)
    {
        // using decision_rule_t = typename sdt::shared_ptr<Action>::value_type;

        // // Get possible histories for all agents
        // auto vect_i_hist = ostate.getAllIndividualHistories();

        // // Get individual decision rules for each agent
        // std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        // for (int ag_id = 0; ag_id < this->getUnderlyingProblem()->getNumAgents(); ag_id++)
        // {
        //     // Generate all individual decision rules for agent 'ag_id'
        //     auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
        //     FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->getUnderlyingProblem()->getActionSpace()->getSpace(ag_id)->getAll());
        //     vect_i_dr.push_back(f_indiv_dr_space.getAll());
        // }

        // // Get joint decision rules for each agent
        // std::vector<sdt::shared_ptr<Action>> vect_j_dr = {};
        // for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        // {
        //     vect_j_dr.push_back(sdt::shared_ptr<Action>(joint_idr));
        // }

        // // Now we can return a discrete space of all joint decision rules
        // return std::make_shared<DiscreteSpace<sdt::shared_ptr<Action>>>(vect_j_dr);
    }

    void OccupancyMDP::initialize(number history_length)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<JointHistory>(this->getUnderlyingProblem()->getNumAgents(), (history_length > 0) ? history_length : -1);

        // Initialize empty state
        this->initial_state_ = std::make_shared<OccupancyState>(this->getUnderlyingProblem()->getNumAgents());
        this->current_state_ = std::make_shared<OccupancyState>(this->getUnderlyingProblem()->getNumAgents());

        // Fill the initial state state
        for (const auto &state : this->getUnderlyingProblem()->getAllStates(0))
        {
            if (this->getUnderlyingProblem()->getStartDistrib()->getProbability(state) > 0)
            {
                const auto &p_x_h = std::make_pair(state, this->initial_history_);
                this->initial_state_->setProbabilityAt(p_x_h, this->getUnderlyingProblem()->getStartDistrib()->getProbability(state));
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> OccupancyMDP::step(sdt::shared_ptr<Action> joint_idr)
    {
        // Select joint action
        const auto &jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

        // Do a step on the DecPOMDP and get next observation and rewards
        const auto &[next_obs, rewards, done] = this->getUnderlyingProblem()->step(jaction);

        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);

        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr);

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done);
    }

    std::shared_ptr<DiscreteSpace<sdt::shared_ptr<Action>>> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate)
    {
        using decision_rule_t = typename sdt::shared_ptr<Action>::value_type;

        // Get individual decision rules for each agent
        std::vector<sdt::shared_ptr<Action>> vect_j_dr = {};
        {

            std::vector<std::vector<decision_rule_t>> vect_i_dr = {};

            {
                // Get possible histories for all agents
                const auto &vect_i_hist = ostate.getAllIndividualHistories();

                for (int ag_id = 0; ag_id < this->getUnderlyingProblem()->getNumAgents(); ag_id++)
                {
                    // Generate all individual decision rules for agent 'ag_id'
                    std::cout << "vect_i_hist[" << ag_id << "].size()=" << vect_i_hist[ag_id].size() << std::endl;
                    const auto &vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
                    FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->getUnderlyingProblem()->getActionSpace()->getSpace(ag_id)->getAll());
                    vect_i_dr.push_back(f_indiv_dr_space.getAll());
                    std::cout << "vect_i_dr[" << ag_id << "].size()=" << vect_i_dr[ag_id].size() << std::endl;
                }
            }
            // Get joint decision rules for each agent
            for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
            {
                vect_j_dr.push_back(sdt::shared_ptr<Action>(joint_idr));
            }
            std::cout << "vect_j_dr.size()=" << vect_j_dr.size() << std::endl;
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<sdt::shared_ptr<Action>>>(vect_j_dr);
    }

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &ostate, const sdt::shared_ptr<Action> &joint_idr, number, std::shared_ptr<HSVI>, bool compression) const
    {
        try
        {
            std::shared_ptr<OccupancyState> occupancy_state = std::static_ptr_cast<OccupancyState>(ostate);
            // The new compressed occupancy state
            std::shared_ptr<OccupancyState> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<OccupancyState> new_fully_uncompressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingProblem()->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<OccupancyState> new_one_step_left_compressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingProblem()->getNumAgents());

            // for all element in the support of the fully uncompressed occupancy state
            for (auto &p_x_o : *occupancy_state->getFullyUncompressedOccupancy())
            {

                auto x = p_x_o.first.first;
                auto o = p_x_o.first.second;

                // Get joint action based on a joint decision rule and a joint labels
                auto jaction = joint_idr.act(occupancy_state->getJointLabels(o->getIndividualHistories()));

                for (auto &y : this->getUnderlyingProblem()->getReachableStates(x, jaction))
                {
                    for (auto &z : this->getUnderlyingProblem()->getReachableObservations(x, jaction, y))
                    {

                        // Get the probability of the next couple (next_state, next_joint history)
                        double next_occupancy_measure = p_x_o.second * this->getUnderlyingProblem()->getObsDynamics()->getDynamics(x, this->getUnderlyingProblem()->getActionSpace()->joint2single(jaction), this->getUnderlyingProblem()->getObsSpace()->joint2single(z), y);

                        // If occupancy measure is greater than zero, we build our occupancy states
                        if (next_occupancy_measure > 0)
                        {
                            // Build fully uncompressed occupancy state
                            auto joint_history_next = o->expand(z);
                            new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, next_occupancy_measure);

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(o);
                            auto compressed_joint_history_next = compressed_joint_history->expand(z);
                            new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, compressed_joint_history_next}, next_occupancy_measure);

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(joint_history_next->getIndividualHistories(), compressed_joint_history_next->getIndividualHistories());
                        }
                    }
                }
            }

            // Finalize the one step left compressed occupancy state
            new_one_step_left_compressed_occupancy_state->finalize();

            if (compression)
            {
                // Compress the occupancy state
                new_compressed_occupancy_state = std::make_shared<std::shared_ptr<State>>(new_one_step_left_compressed_occupancy_state->compress());
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());

                return *new_compressed_occupancy_state;
            }

            new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
            new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
            return *new_one_step_left_compressed_occupancy_state;
        }
        catch (const std::exception &exc)
        {
            std::cout << ostate << std::endl;
            std::cout << joint_idr << std::endl;
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "OccupancyMDP::nextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }

        // try
        // {
        //     // The new compressed occupancy state
        //     std::shared_ptr<std::shared_ptr<State>> new_compressed_occupancy_state;
        //     // The new fully uncompressed occupancy state
        //     std::shared_ptr<std::shared_ptr<State>> new_fully_uncompressed_occupancy_state = std::make_shared<std::shared_ptr<State>>(this->getUnderlyingProblem()->getNumAgents());
        //     // The new one step left occupancy state
        //     std::shared_ptr<std::shared_ptr<State>> new_one_step_left_compressed_occupancy_state = std::make_shared<std::shared_ptr<State>>(this->getUnderlyingProblem()->getNumAgents());

        //     // for all element in the support of the fully uncompressed occupancy state
        //     for (const auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
        //     {

        //         const auto &x = p_x_o.first.first;
        //         const auto &o = p_x_o.first.second;

        //         // Get joint action based on a joint decision rule and a joint labels
        //         const auto &jaction = joint_idr.act(ostate.getJointLabels(o->getIndividualHistories()));

        //         for (const auto &y : this->getUnderlyingProblem()->getReachableStates(x, jaction))
        //         {
        //             for (const auto &z : this->getUnderlyingProblem()->getReachableObservations(x, jaction, y))
        //             {

        //                 // Get the probability of the next couple (next_state, next_joint history)
        //                 double next_proba = p_x_o.second * this->getUnderlyingProblem()->getObsDynamics()->getDynamics(x, this->getUnderlyingProblem()->getActionSpace()->joint2single(jaction), this->getUnderlyingProblem()->getObsSpace()->joint2single(z), y);

        //                 // If occupancy measure is greater than zero, we build our occupancy states
        //                 if (next_proba > 0)
        //                 {
        //                     // Get next joint history from uncompressed joint history
        //                     const auto &next_uncompressed_joint_history = o->expand(z);
        //                     if (this->compress)
        //                     {
        //                         // Get next joint history from joint history labels
        //                         const auto &next_compressed_joint_history = ostate.getCompressedJointHistory(o)->expand(z);

        //                         if (this->keep_fully_uncompressed)
        //                         {
        //                             // Update next history labels
        //                             new_one_step_left_compressed_occupancy_state->updateJointLabels(next_uncompressed_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());

        //                             // Update fully uncompressed occupancy state
        //                             new_fully_uncompressed_occupancy_state->addProbabilityAt({y, next_uncompressed_joint_history}, next_proba);
        //                         }

        //                         // Update one step left uncompressed occupancy state
        //                         new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, next_compressed_joint_history}, next_proba);
        //                     }
        //                     else
        //                     {
        //                         new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, next_uncompressed_joint_history}, next_proba);
        //                     }
        //                 }
        //             }
        //         }
        //     }

        //     // Finalize the one step left compressed occupancy state
        //     new_one_step_left_compressed_occupancy_state->finalize();

        //     if (this->compress)
        //     {
        //         // Compress the occupancy state
        //         new_compressed_occupancy_state = std::make_shared<std::shared_ptr<State>>(new_one_step_left_compressed_occupancy_state->compress());

        //         // Store one step uncompressed
        //         new_compressed_occupancy_state->setOneStepUncompressedOccupancy((this->keep_one_step_uncompressed) ? new_one_step_left_compressed_occupancy_state->getptr() : new_compressed_occupancy_state->getptr());

        //         // Store fully uncompressed
        //         new_compressed_occupancy_state->setFullyUncompressedOccupancy((this->keep_fully_uncompressed) ? new_fully_uncompressed_occupancy_state->getptr() : new_compressed_occupancy_state->getOneStepUncompressedOccupancy());

        //         return *new_compressed_occupancy_state;
        //     }

        //     new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
        //     new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy((this->keep_fully_uncompressed) ? new_fully_uncompressed_occupancy_state->getptr() : new_one_step_left_compressed_occupancy_state->getptr());
        //     return *new_one_step_left_compressed_occupancy_state;
        // }
        // catch (const std::exception &exc)
        // {
        //     std::cout << ostate << std::endl;
        //     std::cout << joint_idr << std::endl;
        //     // catch anything thrown within try block that derives from std::exception
        //     std::cerr << "OccupancyMDP::nextState(..) exception caught: " << exc.what() << std::endl;
        //     exit(-1);
        // }
    }

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &occupancy_state, const sdt::shared_ptr<Action> &joint_idr, number t, std::shared_ptr<HSVI<std::shared_ptr<State>, sdt::shared_ptr<Action>>> hsvi) const
    {
        return this->nextState(occupancy_state, joint_idr, t, hsvi, true);
    }

    double OccupancyMDP::getReward(const std::shared_ptr<State> &ostate, const sdt::shared_ptr<Action> &joint_idr) const
    {
        double r = 0;
        for (const auto &joint_history : ostate->getAllHistories())
        {
            const auto &joint_action = joint_idr.act(joint_history->getIndividualHistories());
            for (const auto &state : ostate->getAllStatesAt(joint_history))
            {
                r += ostate->getProbabilityAt({state, joint_history}) * this->getUnderlyingProblem()->getReward(state, joint_action);
            }
        }
        // double r = 0;
        // for (const auto &p_x_o : ostate)
        // {
        //     const auto &jaction = joint_idr.act(p_x_o.first.second->getIndividualHistories());
        //     r += p_x_o.second * this->getUnderlyingProblem()->getReward()->getReward(p_x_o.first.first, this->getUnderlyingProblem()->getActionSpace()->joint2single(jaction));

        //     // REPLACE BY IN LATER VERSION
        //     // r += p_x_o.second * this->getUnderlyingProblem()->getReward(x, jaction);
        // }

        // for (const auto &occupancy_point : occupancy_state)
        // {
        //     auto joint_history = occupancy_state.getHistory(occupancy_point);
        //     auto belief_state = occupancy_state.getBelief(occupancy_point);
        //     auto joint_action = decision_rule.act(joint_history->getIndividualHistories());
        //     total_reward += (belief_state ^ this->getUnderlyingProblem()->getReward(joint_action));
        // }
        return r;
    }

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &ostate, const sdt::shared_ptr<Action> &joint_idr, number h, std::shared_ptr<HSVI> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    double OccupancyMDP::getExpectedNextValue(std::shared_ptr<ValueFunction> value_function, const std::shared_ptr<State> &ostate, const sdt::shared_ptr<Action> &oaction, number t) const
    {
        return value_function->getValueAt(this->nextState(ostate, oaction), t + 1);
    }

    std::shared_ptr<DiscreteMDP> OccupancyMDP::toMDP()
    {
        return this->getUnderlyingProblem()->toMDP();
    }

    std::shared_ptr<BeliefMDP<BeliefState<>, number, number>> OccupancyMDP::toBeliefMDP()
    {
        return this->getUnderlyingProblem()->toBeliefMDP();
    }

    double OccupancyMDP::getDiscount(number t)
    {
        return this->getUnderlyingProblem()->getDiscount();
    }

    double OccupancyMDP::getWeightedDiscount(number t)
    {
        return std::pow(this->getDiscount(t), t);
    }

    double OccupancyMDP::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number t)
    {
        return std::min(ub - lb, cost_so_far + this->getUnderlyingProblem()->getDiscount() * ub - incumbent) - error / this->getWeightedDiscount(t);
    }

    sdt::shared_ptr<Action> OccupancyMDP::selectNextAction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &state, number t)
    {
        return ub->getBestAction(state, t);
    }

} // namespace sdm
