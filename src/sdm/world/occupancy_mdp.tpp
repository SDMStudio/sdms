#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP()
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
    {
        typename oState::jhistory_type jhist;
        if (hist_length > 0)
        {
            jhist = std::make_shared<typename oState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), hist_length);
        }
        else
        {
            jhist = std::make_shared<typename oState::jhistory_type::element_type>(this->dpomdp_->getNumAgents());
        }

        this->initial_history_ = jhist;
        this->initial_state_ = std::make_shared<oState>();

        for (typename oState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                auto p_x_h = std::make_pair(s, jhist);
                this->initial_state_->setProbabilityAt(p_x_h, this->dpomdp_->getStartDistrib().probabilities()[s]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
        this->current_state_ = this->initial_state_;
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::string underlying_dpomdp, number hist_length) : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename oState, typename oAction>
    oState &OccupancyMDP<oState, oAction>::getState()
    {
        return *this->current_state_;
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::reset()
    {
        this->current_history_ = this->initial_history_;
        this->current_state_ = this->initial_state_;
        this->dpomdp_->reset();
        return *this->current_state_;
    }

    template <typename oState, typename oAction>
    std::tuple<oState, std::vector<double>, bool> OccupancyMDP<oState, oAction>::step(oAction joint_idr)
    {
        auto jaction = joint_idr.act(this->current_history_->getIndividualHistories()); // Select action based on joint separable decision rule
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);           // Do step and get next observation and rewards
        this->current_history_ = this->current_history_->expand(next_obs);                     // Update the history based on the observation
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr);             // Update the occupancy state
        return std::make_tuple(*this->current_state_, rewards, done);                   // return the new occupancy state and the perceived rewards
    }

    template <typename oState, typename oAction>
    bool OccupancyMDP<oState, oAction>::isSerialized() const
    {
        return false;
    }

    template <typename oState, typename oAction>
    DiscreteDecPOMDP *OccupancyMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> OccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        using decision_rule_t = typename oAction::value_type;

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<oAction> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(oAction(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<oAction>>(vect_j_dr);
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &joint_idr, number, std::shared_ptr<HSVI<oState, oAction>>, bool compression) const
    {
        // The new compressed occupancy state
        std::shared_ptr<oState> new_compressed_occupancy_state;
        // The new fully uncompressed occupancy state
        std::shared_ptr<oState> new_fully_uncompressed_occupancy_state = std::make_shared<oState>();
        // The new one step left occupancy state
        std::shared_ptr<oState> new_one_step_left_compressed_occupancy_state = std::make_shared<oState>();

        // for all element in the support of the fully uncompressed occupancy state
        for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;

            // Get joint action based on a joint decision rule and a joint labels
            auto jaction = joint_idr.act(ostate.getJointLabels(o->getIndividualHistories()));

            for (auto &y : this->dpomdp_->getReachableStates(x, jaction))
            {
                for (auto &z : this->dpomdp_->getReachableObservations(x, jaction, y))
                {
                    // Get the probability of the next couple (next_state, next_joint history)
                    double next_occupancy_measure = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(jaction), this->dpomdp_->getObsSpace()->joint2single(z), y);

                    // If occupancy measure is greater than zero, we build our occupancy states
                    if (next_occupancy_measure > 0)
                    {
                        // Build fully uncompressed occupancy state
                        auto joint_history_next = o->expand(z);
                        new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, next_occupancy_measure);

                        // Build one step left uncompressed occupancy state
                        auto compressed_joint_history = ostate.getCompressedJointHistory(o);
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
            new_compressed_occupancy_state = std::make_shared<oState>(new_one_step_left_compressed_occupancy_state->compress());
            new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
            new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
            return *new_compressed_occupancy_state;
        }

        new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
        new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
        return *new_one_step_left_compressed_occupancy_state;
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &joint_idr, number h, std::shared_ptr<HSVI<oState, oAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            auto jaction = joint_idr.act(o->getIndividualHistories());
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));

            // REPLACE BY IN LATER VERSION
            // r += p_x_o.second * this->dpomdp_->getReward(x, jaction);
        }
        return r;
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<oState, oAction>> value_function, const oState &ostate, const oAction &oaction, number t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteMDP> OccupancyMDP<oState, oAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> OccupancyMDP<oState, oAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

} // namespace sdm
