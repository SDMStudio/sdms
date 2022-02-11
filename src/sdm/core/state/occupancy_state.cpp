#include <iomanip>
#include <algorithm>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>

namespace sdm
{
    double OccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;

    RecursiveMap<Joint<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> OccupancyState::jhistory_map_ = {};

    OccupancyState::OccupancyState() : OccupancyState(2, 0)
    {
    }

    OccupancyState::OccupancyState(number num_agents, number h) : OccupancyState(num_agents, h, COMPRESSED) {}

    OccupancyState::OccupancyState(number num_agents, number h, StateType stateType) : Belief(), state_type(stateType), num_agents_(num_agents), h(h), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())

    {
        for (number agent_id = 0; agent_id < num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->weight_of_private_occupancy_state_.push_back({});
            this->private_ihistory_map_.push_back({});
            //
            this->individual_hierarchical_history_vector_map_vector.push_back(std::make_shared<std::unordered_map<number, std::vector<std::shared_ptr<JointHistoryInterface>>>>());
        }
        this->joint_history_map_vector = std::make_shared<std::unordered_map<number, std::vector<std::shared_ptr<JointHistoryInterface>>>>();
    }

    OccupancyState::OccupancyState(const OccupancyState &occupancy_state)
        : Belief(occupancy_state),
          num_agents_(occupancy_state.num_agents_),
          h(occupancy_state.h),
          state_type(occupancy_state.state_type),
          tuple_of_maps_from_histories_to_private_occupancy_states_(occupancy_state.tuple_of_maps_from_histories_to_private_occupancy_states_),
          weight_of_private_occupancy_state_(occupancy_state.weight_of_private_occupancy_state_),
          fully_uncompressed_occupancy_state(occupancy_state.fully_uncompressed_occupancy_state),
          one_step_left_compressed_occupancy_state(occupancy_state.one_step_left_compressed_occupancy_state),
          compressed_occupancy_state(occupancy_state.compressed_occupancy_state),
          private_ihistory_map_(occupancy_state.private_ihistory_map_),
          probability_ihistories(occupancy_state.probability_ihistories),
          list_beliefs_(occupancy_state.list_beliefs_),
          list_joint_histories_(occupancy_state.list_joint_histories_),
          all_list_ihistories_(occupancy_state.all_list_ihistories_),
          map_joint_history_to_belief_(occupancy_state.map_joint_history_to_belief_),
          ihistories_to_jhistory_(occupancy_state.ihistories_to_jhistory_),
          action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>()),
          individual_hierarchical_history_vector_map_vector(occupancy_state.individual_hierarchical_history_vector_map_vector),
          joint_history_map_vector(occupancy_state.joint_history_map_vector)
    {
    }

    OccupancyState::~OccupancyState()
    {
    }

    std::shared_ptr<OccupancyState> OccupancyState::make(number h)
    {
        return std::make_shared<OccupancyState>(this->num_agents_, h, this->state_type);
    }

    std::shared_ptr<OccupancyState> OccupancyState::copy()
    {
        return std::make_shared<OccupancyState>(*this);
    }

    void OccupancyState::setStateType(const StateType &state_type)
    {
        this->state_type = state_type;
    }

    double OccupancyState::getProbability(const std::shared_ptr<State> &joint_history) const
    {
        return Belief::getProbability(joint_history);
    }

    double OccupancyState::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return Belief::getProbability(joint_history);
    }

    double OccupancyState::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &state) const
    {
        // Get the probability p(x,o) = p(o) * b(x | o)
        auto belief = this->getBeliefAt(joint_history);
        auto output = (belief == nullptr) ? 0. : Belief::getProbability(joint_history) * belief->getProbability(state);
        return output;
    }

    void OccupancyState::setProbability(const std::shared_ptr<State> &joint_history, double proba)
    {
        Belief::setProbability(joint_history, proba);
    }

    void OccupancyState::setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        // Set the belief corresponding to a specific joint history
        this->setBeliefAt(joint_history, belief);

        // Set the probability of the joint history
        Belief::setProbability(joint_history, proba);

        // this->setProbability(joint_history, proba);
    }

    void OccupancyState::addProbability(const std::shared_ptr<State> &joint_history, double proba)
    {
        // Add the probability of being in a joint history
        this->setProbability(joint_history, this->getProbability(joint_history) + proba);
    }

    void OccupancyState::addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        // Get the corresponding belief of an history. This will return nullptr if no such history exists
        auto corresponding_belief = this->getBeliefAt(joint_history);

        // Get the belief label (corresponding belief or inputed belief)
        auto belief_label = (corresponding_belief != nullptr) ? corresponding_belief : belief;

        // Add input probability to the current probability
        this->setProbability(joint_history, belief_label, this->getProbability(joint_history) + proba);
    }

    bool OccupancyState::checkCompatibility(const std::shared_ptr<Observation> &, const std::shared_ptr<Observation> &)
    {
        return true;
    }

    std::shared_ptr<Action> OccupancyState::applyDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history)
    {
        return dr->act(joint_history);
    }

    Pair<std::shared_ptr<State>, double> OccupancyState::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // The new one step left occupancy state
        auto next_one_step_left_compressed_occupancy_state = this->make(t + 1);
        auto decision_rule = action->toDecisionRule();
        auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(mdp);

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &compressed_joint_history : this->getJointHistories())
        {
            // Get p(o_t)
            double proba_history = this->getProbability(compressed_joint_history);

            // Get the corresponding belief
            auto belief = this->getBeliefAt(compressed_joint_history);

            // Apply decision rule and get action
            auto jaction = this->applyDR(decision_rule, compressed_joint_history); // this->act(compressed_joint_history);

            // For each action that is likely to be taken
            for (const auto &joint_action : {jaction}) // decision_rule->getDistribution(compressed_joint_history)->getSupport())
            {
                // Get p(u_t | o_t)
                double proba_action = 1; // decision_rule->getProbability(compressed_joint_history, joint_action);

                // For each observation in the space of joint observation
                for (auto jobs : *pomdp->getObservationSpace(t))
                {
                    auto joint_observation = jobs->toObservation();
                    if (this->checkCompatibility(joint_observation, observation))
                    {
                        // Get the next belief and p(z_{t+1} | b_t, u_t)
                        auto [next_belief, proba_observation] = belief->next(mdp, joint_action, joint_observation, t);

                        double next_joint_history_probability = proba_history * proba_action * proba_observation;

                        // If the next history probability is not zero
                        if (next_joint_history_probability > 0)
                        {
                            // Update new one step uncompressed occupancy state
                            std::shared_ptr<JointHistoryInterface> next_compressed_joint_history = compressed_joint_history->expand(joint_observation /*, joint_action*/)->toJointHistory();
                            this->updateOccupancyStateProba(next_one_step_left_compressed_occupancy_state, next_compressed_joint_history, next_belief->toBelief(), next_joint_history_probability);
                        }
                    }
                }
            }
        }

        return this->finalizeNextState(next_one_step_left_compressed_occupancy_state, t);
    }

    Pair<std::shared_ptr<OccupancyStateInterface>, double> OccupancyState::finalizeNextState(const std::shared_ptr<OccupancyStateInterface> &one_step_occupancy_state, number t)
    {
        // Finalize and normalize the one step left occupancy state
        one_step_occupancy_state->finalize();
        double norm_one_step = one_step_occupancy_state->norm_1();
        one_step_occupancy_state->normalizeBelief(norm_one_step);

        if (this->state_type == StateType::FULLY_UNCOMPRESSED)
        {
            return {one_step_occupancy_state, norm_one_step};
        }
        else
        {
            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> compressed_occupancy_state;

            // Compress the occupancy state
            compressed_occupancy_state = one_step_occupancy_state->compress();

            double norm_compressed = compressed_occupancy_state->norm_1();
            compressed_occupancy_state->normalizeBelief(norm_compressed);
            compressed_occupancy_state->setOneStepUncompressedOccupancy(one_step_occupancy_state);
            one_step_occupancy_state->setCompressedOccupancy(compressed_occupancy_state);

            switch (this->state_type)
            {
            case StateType::COMPRESSED:
                return {compressed_occupancy_state, norm_one_step};
            case StateType::ONE_STEP_UNCOMPRESSED:
                return {one_step_occupancy_state, norm_one_step};
            default:
                throw sdm::exception::TypeError("StateType not recognized.");
                break;
            }
        }
    }

    void OccupancyState::updateOccupancyStateProba(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability)
    {
        if (occupancy_state->getProbability(joint_history) > 0.)
        {
            // Get the probability of being in each belief
            double proba_belief1 = occupancy_state->getProbability(joint_history), proba_belief2 = probability;
            // Cast to belief structure
            std::shared_ptr<Belief> belief1 = std::dynamic_pointer_cast<Belief>(occupancy_state->getBeliefAt(joint_history)), belief2 = std::dynamic_pointer_cast<Belief>(belief);

            // Aggregate beliefs
            std::shared_ptr<Belief> aggregated_belief = std::make_shared<Belief>(belief1->add(*belief2, proba_belief1, proba_belief2));

            // Normalize the resulting belief
            aggregated_belief->normalizeBelief(aggregated_belief->norm_1());

            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history, aggregated_belief, occupancy_state->getProbability(joint_history) + probability);
        }
        else
        {
            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history, belief, probability);
        }
    }

    double OccupancyState::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    {
        double reward = 0.;
        auto decision_rule = action->toDecisionRule();
        // For all histories in the occupancy state
        for (const auto &jhist : this->getJointHistories())
        {
            // Get the belief corresponding to this history
            auto belief = this->getBeliefAt(jhist);
            // Get the action from decision rule
            auto joint_action = this->applyDR(decision_rule, jhist); // decision_rule->act(jhist);

            // Update the expected reward
            reward += this->getProbability(jhist) * belief->getReward(mdp, joint_action, t);
        }
        return reward;
    }

    Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> OccupancyState::sampleJointHistoryBelief()
    {
        auto sampled_joint_history = this->sampleState()->toHistory()->toJointHistory();
        return std::make_pair(sampled_joint_history, this->getBeliefAt(sampled_joint_history));
    }

    size_t OccupancyState::hash(double precision) const
    {
        if (precision < 0)
        {
            precision = OccupancyState::PRECISION;
        }

        size_t seed = 0;
        double inverse_of_precision = 1. / precision;
        std::map<std::shared_ptr<sdm::State>, double> ordered(this->container.begin(), this->container.end());
        std::vector<int> rounded;
        for (const auto &pair_jhist_proba : ordered)
        {
            sdm::hash_combine(seed, pair_jhist_proba.first);
            rounded.push_back(lround(inverse_of_precision * pair_jhist_proba.second));
        }
        for (const auto &v : rounded)
        {
            // Combine the hash of the current vector with the hashes of the previous ones
            sdm::hash_combine(seed, v);
        }
        return seed;
    }

    bool OccupancyState::operator==(const OccupancyState &other) const
    {
        return this->isEqual(other, OccupancyState::PRECISION);
    }

    bool OccupancyState::isEqualNorm1(const std::shared_ptr<BeliefInterface> &other, double precision) const
    {
        // return sdm::Norm1Equality(left, right, PWLCQValueFunction::GRANULARITY);
        double norm_1 = 0., additional = 1., proba_right;
        auto other_copy = other->toOccupancyState();
        // For all points in the support
        for (const auto &jhistory : this->getJointHistories())
        {
            // For all states in the corresponding belief
            for (const auto &state : this->getBeliefAt(jhistory)->getStates())
            {
                proba_right = other_copy->getProbability(jhistory, state);
                additional -= proba_right;
                norm_1 += std::abs(this->getProbability(jhistory, state) - proba_right);
                if (norm_1 > precision)
                    return false;
            }
        }

        return (((norm_1 + additional) / 2) - 1e-5 <= precision);
    }

    bool OccupancyState::isEqualNormInf(const std::shared_ptr<BeliefInterface> &other, double precision) const
    {
    }

    bool OccupancyState::isEqual(const OccupancyState &other, double precision) const
    {
        // return this->isEqualNormInf(other, precision);

        if (precision < 0)
        {
            precision = Belief::PRECISION;
        }

        // if (this->size() != other.size())
        // {
        //     return false;
        // }

        // if (std::abs(this->getDefault() - other.getDefault()) > precision)
        // {
        //     return false;
        // }

        for (const auto &jhistory : this->getJointHistories())
        {
            // For all states in the corresponding belief
            for (const auto &state : this->getBeliefAt(jhistory)->getStates())
            {
                // Does the corresponding probabilities are equals ?
                if (std::abs(this->getProbability(jhistory, state) - other.getProbability(jhistory, state)) > precision)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool OccupancyState::isEqual(const std::shared_ptr<State> &other, double precision) const
    {
        return this->isEqual(*std::dynamic_pointer_cast<OccupancyState>(other), precision);
    }

    bool OccupancyState::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->isEqual(*std::dynamic_pointer_cast<OccupancyState>(other));
    }

    double OccupancyState::product(const std::shared_ptr<AlphaVector> &alpha)
    {
        double product = 0.0;

        for (const auto &jhistory : this->getJointHistories())
        {
            for (const auto &state : this->getBeliefAt(jhistory)->getStates())
            {
                product += this->getProbability(jhistory, state) * alpha->getValueAt(state, jhistory);
            }
        }
        return product;
    }

    double OccupancyState::product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action)
    {

        double product = 0.0;
        auto decision_rule = action->toDecisionRule();
        for (auto history : this->getJointHistories())
        {
            auto action = this->applyDR(decision_rule, history);
            double proba_a = 1;

            for (auto state : this->getBeliefAt(history)->getStates())
            {
                product += this->getProbability(history, state) * proba_a * beta->getValueAt(state, history, action);
            }
        }
        return product;
    }

    // ###################################
    // ###### MANIPULATE DATA ############
    // ###################################

    const std::set<std::shared_ptr<JointHistoryInterface>> &OccupancyState::getJointHistories() const
    {
        return this->list_joint_histories_;
    }

    const std::set<std::shared_ptr<BeliefInterface>> &OccupancyState::getBeliefs() const
    {
        return this->list_beliefs_;
    }

    std::shared_ptr<BeliefInterface> OccupancyState::getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        auto iterator_on_belief = this->map_joint_history_to_belief_.find(jhistory);
        return (iterator_on_belief == this->map_joint_history_to_belief_.end()) ? nullptr : iterator_on_belief->second;
    }

    void OccupancyState::setBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory, const std::shared_ptr<BeliefInterface> &belief)
    {
        this->map_joint_history_to_belief_[jhistory] = belief;
    }

    const std::set<std::shared_ptr<HistoryInterface>> &OccupancyState::getIndividualHistories(number agent_id) const
    {
        return this->all_list_ihistories_[agent_id];
    }

    const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &OccupancyState::getAllIndividualHistories() const
    {
        return this->all_list_ihistories_;
    }

    void OccupancyState::setupIndividualHistories()
    {
        // this->all_list_ihistories_.clear();
        bool first_passage = true;
        for (const auto &jhist : this->getJointHistories())
        {
            const auto &ihists = jhist->getIndividualHistories();
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                if (first_passage)
                {
                    this->all_list_ihistories_.push_back({});
                }
                this->all_list_ihistories_[i].insert(ihists[i]);
            }
            first_passage = false;
        }
    }

    void OccupancyState::setupBeliefsAndHistories()
    {
        // Get the set of joint histories that are in the support of the OccupancyState
        // this->list_joint_histories_.clear();
        // this->list_beliefs_.clear();
        for (const auto &joint_history_tmp : this->getStates())
        {
            auto joint_history = std::dynamic_pointer_cast<JointHistoryInterface>(joint_history_tmp);
            this->list_joint_histories_.insert(joint_history);
            this->list_beliefs_.insert(this->getBeliefAt(joint_history));
        }
    }

    void OccupancyState::setup()
    {
        this->setupBeliefsAndHistories();
        this->setupIndividualHistories();
    }

    // #############################################
    // ###### MANIPULATE REPRESENTATION ############
    // #############################################

    const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &OccupancyState::getPrivateOccupancyStates() const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    }

    const std::shared_ptr<PrivateOccupancyState> &OccupancyState::getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_.at(agent_id).at(ihistory);
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getFullyUncompressedOccupancy()
    {
        // If fully uncompressed is nullptr then we consider fully uncompressed is equal to this
        return (this->fully_uncompressed_occupancy_state) ? this->fully_uncompressed_occupancy_state : this->getptr();
    }

    void OccupancyState::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_ostate)
    {
        if (fully_uncompressed_ostate != this->getptr())
            this->fully_uncompressed_occupancy_state = fully_uncompressed_ostate;
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getOneStepUncompressedOccupancy()
    {
        // If one step uncompressed is nullptr then we consider one step uncompressed is equal to this
        return (this->one_step_left_compressed_occupancy_state) ? this->one_step_left_compressed_occupancy_state : this->getptr();
    }

    void OccupancyState::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &one_step_uncompress_ostate)
    {
        if (one_step_uncompress_ostate != this->getptr())
            this->one_step_left_compressed_occupancy_state = one_step_uncompress_ostate;
        // std::dynamic_pointer_cast<OccupancyState>(this->getOneStepUncompressedOccupancy())->setCompressedOccupancy(this->getptr());
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getCompressedOccupancy()
    {
        return (!this->compressed_occupancy_state.expired()) ? this->compressed_occupancy_state.lock() : this->getptr();
    }

    void OccupancyState::setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate)
    {
        this->compressed_occupancy_state = compress_ostate;
    }

    // #####################################
    // ###### MANIPULATE LABELS ############
    // #####################################

    std::shared_ptr<HistoryInterface> OccupancyState::getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const
    {
        auto iterator = this->private_ihistory_map_.at(agent_id).find(ihistory);
        return (iterator == this->private_ihistory_map_.at(agent_id).end()) ? ihistory : iterator->second;
    }

    Joint<std::shared_ptr<HistoryInterface>> OccupancyState::getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories) const
    {
        Joint<std::shared_ptr<HistoryInterface>> new_list_ihistories;
        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // if the ihistory was never compressed
            new_list_ihistories.push_back(this->getLabel(list_ihistories.at(agent_id), agent_id));
        }
        return new_list_ihistories;
    }

    void OccupancyState::updateLabel(number agent_id, const std::shared_ptr<HistoryInterface> &ihistory, const std::shared_ptr<HistoryInterface> &label)
    {
        this->private_ihistory_map_[agent_id][ihistory] = label;
        if (ihistory != label)
        {
            for (const auto &pair_ihistory_label : this->private_ihistory_map_[agent_id])
            {
                if (pair_ihistory_label.second == ihistory)
                {
                    this->updateLabel(agent_id, pair_ihistory_label.first, label);
                }
            }
        }
    }

    void OccupancyState::updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories, const Joint<std::shared_ptr<HistoryInterface>> &list_labels)
    {
        for (number agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            this->updateLabel(agent_id, list_ihistories.at(agent_id), list_labels.at(agent_id));
        }
    }

    // #############################################
    // ######### MANIPULATE COMPRESSION ############
    // #############################################

    std::shared_ptr<JointHistoryInterface> OccupancyState::getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        const auto &labels = this->getJointLabels(joint_history->getIndividualHistories());
        return this->jhistory_map_.at(labels);
    }

    bool OccupancyState::areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &ihistory_1, const std::shared_ptr<HistoryInterface> &ihistory_2, number agent_identifier)
    {
        return this->getPrivateOccupancyState(agent_identifier, ihistory_1)->check_equivalence(*this->getPrivateOccupancyState(agent_identifier, ihistory_2));
    }

    /**
     * @brief
     *
     * https://gitlab.inria.fr/maintenance/maintenance.html?appli=GITLAB
     * @return std::shared_ptr<OccupancyStateInterface>
     */
    std::shared_ptr<OccupancyStateInterface> OccupancyState::compress()
    {

        auto current_compact_ostate = this->make(this->h);
        auto previous_compact_ostate = this->copy();

        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // Get support (a set of individual histories for agent i)
            const auto &support_set = this->getIndividualHistories(agent_id);
            auto &&support = tools::set2vector(support_set);

            // Sort support
            std::sort(support.begin(), support.end());

            for (auto iter_first = support.begin(); iter_first != support.end();)
            {
                auto ihistory_label = *iter_first;      // Get the ihistory "label"
                iter_first = support.erase(iter_first); // Erase the ihistory "label" from the support

                // Set probability of labels
                for (const auto &joint_history : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistories())
                {
                    auto belief = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getBeliefAt(joint_history);
                    current_compact_ostate->setProbability(joint_history, belief, previous_compact_ostate->getProbability(joint_history));
                }

                // For all other individual histories in the support
                for (auto iter_second = iter_first; iter_second != support.end();)
                {
                    // Get the ihistory we want check the equivalence
                    auto ihistory_one_step_left = *iter_second;

                    // Check equivalence between individual histories
                    if (this->areIndividualHistoryLPE(ihistory_label, ihistory_one_step_left, agent_id))
                    {
                        // If ihistories are equivalent
                        // Store the new label
                        this->updateLabel(agent_id, ihistory_one_step_left, ihistory_label);

                        // Erase unecessary equivalent individual history
                        iter_second = support.erase(iter_second);

                        // ----- Update probability of the new compact occupancy state by adding proba of the equivalent ihistory ---
                        // For all private joint history in the previous compact occupancy state
                        for (const auto &private_joint_history : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getJointHistories())
                        {
                            // Get the probability of the private occupancy state corresponding to the history that will be deleted
                            double probability = this->weight_of_private_occupancy_state_[agent_id][ihistory_one_step_left] * previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getProbability(private_joint_history);

                            // Get the partial joint history label
                            auto partial_jhist = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getPartialJointHistory(private_joint_history);

                            // Get the joint history corresponding to this partial joint history in the private occupancy state of the history label
                            auto joint_history_from_partial = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistoryFromPartial(partial_jhist);

                            // Update the current compact occupancy state
                            current_compact_ostate->addProbability(joint_history_from_partial,
                                                                   previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getBeliefAt(joint_history_from_partial),
                                                                   probability);
                        }
                    }
                    else
                    {
                        iter_second++;
                    }
                }
            }

            *previous_compact_ostate = *current_compact_ostate;
            previous_compact_ostate->private_ihistory_map_ = this->private_ihistory_map_;
            previous_compact_ostate->finalize();
            current_compact_ostate->container.clear();
        }

        // previous_compact_ostate->setFullyUncompressedOccupancy(this->getFullyUncompressedOccupancy());
        // previous_compact_ostate->setOneStepUncompressedOccupancy(this->getptr());

        return previous_compact_ostate;
    }

    void OccupancyState::setupPrivateOccupancyStates()
    {
        // For all joint histories in the support of the occupancy state
        for (const auto &jhist : this->getJointHistories())
        {
            // Get the probability of being in this history
            const auto &proba = this->getProbability(jhist);

            // Get the corresponding belief
            auto belief = this->getBeliefAt(jhist);

            // Store relation between joint history and list of individual histories
            this->jhistory_map_.emplace(jhist->getIndividualHistories(), jhist);

            // For each agent we update its private occupancy state
            for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
            {
                // Instanciation empty private occupancy state associated to ihistory and agent i if not exists
                if (this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].find(jhist->getIndividualHistory(agent_id)) == this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].end())
                {
                    this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].emplace(jhist->getIndividualHistory(agent_id), std::make_shared<PrivateOccupancyState>(agent_id, this->num_agents_, this->h));
                }
                // Set private occupancy measure
                this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(jhist, belief, proba);
            }
        }

        // ----------- FINALIZE PRIVATE OCCUPANCY STATES --------------
        // For all agents
        for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
        {
            // For all individual histories
            for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
            {
                // Finalize the private occupancy state
                pair_ihist_private_occupancy_state.second->finalize(false);

                // Get the weight of a private occupancy state
                this->weight_of_private_occupancy_state_[agent_id][pair_ihist_private_occupancy_state.first] = pair_ihist_private_occupancy_state.second->norm_1();

                // Normalize the private occupancy state
                pair_ihist_private_occupancy_state.second->normalize();
            }
        }
    }

    void OccupancyState::finalize()
    {
        Belief::finalize();
        this->setup();
        this->setupPrivateOccupancyStates();
        this->setProbabilityOverIndividualHistories();
    }

    void OccupancyState::finalize(bool do_compression)
    {
        if (do_compression)
        {
            finalize();
        }
        else
        {
            Belief::finalize();
            this->setup();
        }
    }

    void OccupancyState::normalize()
    {
        double sum = this->norm_1();
        for (const auto &joint_history : this->getJointHistories())
        {
            this->setProbability(joint_history, this->getBeliefAt(joint_history), this->getProbability(joint_history) / sum);
        }
    }

    std::shared_ptr<OccupancyState> OccupancyState::getptr()
    {
        return std::dynamic_pointer_cast<OccupancyState>(this->getPointer());
    }

    std::string OccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<occupancy-state size=\"" << this->size() << "\">\n";
        for (const auto &history_as_state : this->getStates())
        {
            auto joint_history = history_as_state->toHistory()->toJointHistory();
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</occupancy-state>";
        return res.str();
    }

    double OccupancyState::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->probability_ihistories.at(agent).at(ihistory);
    }

    void OccupancyState::setProbabilityOverIndividualHistories()
    {
        // For all agents
        for (number ag_id = 0; ag_id < this->num_agents_; ag_id++)
        {
            // For all individual history of this agent
            for (const auto &ihistory : this->all_list_ihistories_[ag_id])
            {
                // Compute the probability of the individual history of agent i
                double prob = 0.0;
                for (const auto &jhistory : this->getPrivateOccupancyState(ag_id, ihistory)->getStates())
                {
                    prob += this->getProbability(jhistory);
                }
                this->probability_ihistories[ag_id][ihistory] = prob;
            }
        }
    }

    // #############################################
    // ######### ACTION SPACE ######################
    // #############################################

    std::shared_ptr<Space> OccupancyState::getActionSpaceAt(number t)
    {
        if (this->action_space_map->find(t) != this->action_space_map->end())
        {
            return this->action_space_map->at(t);
        }
        else
        {
            return nullptr;
        }
    }

    void OccupancyState::setActionSpaceAt(number t, std::shared_ptr<Space> action_space)
    {
        this->action_space_map->emplace(t, action_space);
    }

    // #############################################
    // ######### PHOS ##############################
    // #############################################

    std::shared_ptr<JointHistoryInterface> OccupancyState::getJointHistory(std::shared_ptr<JointHistoryInterface> candidate_jhistory)
    {
        for (const std::shared_ptr<JointHistoryInterface> &joint_history : this->getJointHistories())
        {
            if (*std::dynamic_pointer_cast<JointHistoryTree>(joint_history) == *std::dynamic_pointer_cast<JointHistoryTree>(candidate_jhistory))
            {
                return joint_history;
            }
        }
        return nullptr;
    }

} // namespace sdm
