#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::HierarchicalTabularQValueFunction(
        number horizon, double learning_rate, std::shared_ptr<QInitializer<typename TState::second_type, TAction>> initializer
    ) : QValueFunction<TState, TAction, TValue>(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {}

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::HierarchicalTabularQValueFunction(
        number horizon, double learning_rate, std::shared_ptr<QInitializer<typename TState::second_type, TAction>> initializer, std::shared_ptr<HierarchicalPrivateOccupancyMDP<typename TState::first_type, HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<number>>, number>>> env
    ) : QValueFunction<TState, TAction, TValue>(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer), env_(env)
    {}

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::HierarchicalTabularQValueFunction(number horizon, double learning_rate, TValue default_value) 
      : HierarchicalTabularQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {}

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::initialize()
    {   
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::initialize(TValue default_value, number t)
    {   
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    std::shared_ptr<VectorImpl<TAction, TValue>> HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValuesAt(const TState &state, number t)
    {
        this->initializeIfNeeded(state);
        Joint<HistoryTree_p<number>> o = get_o(state.first, state.second);
        return this->representation.at(state.first).getQValuesAt(o, t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    std::shared_ptr<VectorImpl<TAction, TValue>> HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValuesAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, number t)
    {
        return this->getQValuesAt(std::make_pair(private_occupancy_state, history), t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TValue HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValueAt(const TState &state, const TAction &action, number t)
    {   
        this->initializeIfNeeded(state);
        Joint<HistoryTree_p<number>> o = get_o(state.first, state.second);
        return this->representation.at(state.first).getQValueAt(o, action, t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TValue HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, const TAction &action, number t)
    {   
        return this->getQValueAt(std::make_pair(private_occupancy_state, history), action, t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TAction HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getBestAction(const TState &state, number t)
    {   
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<observation>>, action> HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getGreedyAction(const TState &state, number t)
    {   
        // Number of agents.
        number N = this->env_->getUnderlyingProblem()->getNumAgents();

        // Get agent N's private occupancy state
        PrivateOccupancyState<number, JointHistoryTree_p<observation>> sN = state.first;
        // // Get agent N's history which is public
        // JointHistoryTree_p<number> real_o = state.second;

        // The joint precalculated decision rule, for all agents except agent N.
        std::map<agent, DeterministicDecisionRule<Pair<Joint<HistoryTree_p<observation>>, Joint<action>>, action>> b;

        // For all agents between 1 and N-1, we need to obtain their precalculated decision rules.
        for (agent i = 0; i < N - 1; i++)
        {   
            // Accessible states for the b of agent I.
            std::vector<Pair<Joint<HistoryTree_p<observation>>, Joint<action>>> acc_states_bI;
            // Actions for each of these acc_states_bI.
            std::vector<action> n_actions_bI;
            // For each support (?) of sN. This forst support will be used to get all possible histories for agent I.
            for(auto &x_o__p: sN)
            {   
                // This joint partial history will have histories of agents between I and N, that is those visible to agent I.
                Joint<HistoryTree_p<number>> partial_o;
                for (int m = i; m < N; m++)
                {   
                    partial_o.push_back(x_o__p.first.second->at(m));
                }
                // For all possible actions of lower ranked agents:
                for (const Joint<action> &lower_ranked_agents_jaction_reversed : this->env_->get_vector_lower_ranked_agents_jactions_reversed(i))
                {   
                    // std::cout << "lower_ranked_agents_jaction_reversed " << lower_ranked_agents_jaction_reversed << std::endl;
                    // Push the input.
                    acc_states_bI.push_back(std::make_pair(partial_o, lower_ranked_agents_jaction_reversed));
                    // Q values for actions of Agent I.
                    std::map<typename TAction::value_type, double> q_values_uI;
                    // For all actions of Agent I:
                    for(const action & uI: this->env_->getUnderlyingProblem()->getActionSpace()->getSpace(i)->getAll())
                    {   
                        // Initialize the QValue associated to uI to 0.
                        q_values_uI[uI] = 0;
                        // This secondary support will be used to estimate the Q Values for uI.
                        for(auto &y_w__r: sN)
                        {
                            // Initialize the partial joint action which has all actions between of agents N and I+1 in reverse order.
                            Joint<action> partial_u_reversed = lower_ranked_agents_jaction_reversed;
                            // Now partial_u_reversed has all actions of agents between N and I.
                            partial_u_reversed.push_back(uI);
                            // For agents between I-1 and 0, that is for agents who are not visible to agent I:
                            for (int j = i - 1; j >= 0; j--)
                            {   
                                // Initialize the partial joint history which will have all histories of agents between agent J and N.
                                Joint<HistoryTree_p<number>> partial_o;
                                for (int m = j; m < N; m++)
                                {
                                    partial_o.push_back(y_w__r.first.second->at(m));
                                }
                                // Get precalculated decision rule of agent J.
                                DeterministicDecisionRule<Pair<Joint<HistoryTree_p<number>>, Joint<number>>, number> bJ = b.at(j);
                                // Using bJ and partial_o (which has all oM, for j<=m<=N) get uJ.
                                action uJ = bJ(std::make_pair(partial_o, partial_u_reversed));
                                // Add uJ to partial_j_reversed
                                partial_u_reversed.push_back(uJ);
                            }
                            // Now partial_u_reversed should have all actions for agents N to 1.
                            // Initialize Joint Action.
                            Joint<action> u;
                            for (int e = partial_u_reversed.size() - 1; e >= 0; e--){
                                u.push_back(partial_u_reversed.at(e));
                            }
                            // Construct the JointHistory.
                            Joint<HistoryTree_p<number>> o;
                            // For each agent between 1 to N:
                            for (agent a = 0; a < N; a++){
                                o.push_back(y_w__r.first.second->at(a));
                            }
                            q_values_uI[uI] = this->getQValueAt(sN, o, u, t);
                        }
                    }
                    // action greedy_uI = q_values_uI->argmax();
                    auto greedy_uI_it  = std::max_element(
                        q_values_uI.begin(), 
                        q_values_uI.end(),
                        [](const std::pair<typename TAction::value_type, double>& p1, const std::pair<typename TAction::value_type, double>& p2)
                        {
                            return p1.second < p2.second; 
                        }
                    );
                    action greedy_uI = greedy_uI_it->first;
                    // Push the output.
                    n_actions_bI.push_back(greedy_uI);
                }
            }
            // Initialize b of agent I
            DeterministicDecisionRule<Pair<Joint<HistoryTree_p<observation>>, Joint<action>>, action> bI(acc_states_bI, n_actions_bI);
            // Push bI to b.
            b.emplace(i, bI);
        }

        // Q values for actions of Agent N.
        std::map<typename TAction::value_type, double> q_values_uN;
        // For each possible uN:
        for(const auto & uN: this->env_->getUnderlyingProblem()->getActionSpace()->getSpace(N - 1)->getAll())
        {   
            // std::cout << "uN " << uN << std::endl;
            // Initialize it to 0.
            q_values_uN[uN] = 0;
            // For each support (?) of s2:
            for(auto &x_o__p: sN)
            {
                Joint<HistoryTree_p<number>> o;
                for(agent i = 0; i < N; i++)
                {   
                    // std::cout << "i " << i << std::endl;
                    o.push_back(x_o__p.first.second->at(i));
                }
                Joint<action> u_reversed;
                u_reversed.push_back(uN);
                for (int j = N - 2; j >= 0; j--)
                {   
                    // std::cout << "j " << j << std::endl;
                    Joint<HistoryTree_p<number>> partial_o;
                    for (int k = j; k < N; k++)
                    {   
                        // std::cout << "k " << k << std::endl;
                        partial_o.push_back(x_o__p.first.second->at(k));
                    }
                    // Get precalculated decision rule of agent J.
                    DeterministicDecisionRule<Pair<Joint<HistoryTree_p<observation>>, Joint<action>>, action> bJ = b.at(j);
                    // Using bJ and partial_o (which has all oM, for j<=m<=N) get uJ.
                    action uJ = bJ(std::make_pair(partial_o, u_reversed));
                    u_reversed.push_back(uJ);
                }
                Joint<action> u;
                for (int a = u_reversed.size() - 1; a >= 0; a--)
                {   
                    // std::cout << "a" << a << std::endl;
                    u.push_back(u_reversed.at(a));
                }
                // Increase uN's Q value given all that and probability of this support (?).
                q_values_uN[uN] += x_o__p.second * this->getQValueAt(sN, o, u, t);   
            }
        }
        // number greedy_uN = q_values_uN->argmax();
        auto greedy_uN_it  = std::max_element(
            q_values_uN.begin(), 
            q_values_uN.end(),
            [](const std::pair<typename TAction::value_type, double>& p1, const std::pair<typename TAction::value_type, double>& p2)
                {
                    return p1.second < p2.second; 
                }
        );
        number greedy_uN = greedy_uN_it->first;

        // The joint decision rule, except for agent N.
        std::vector<DeterministicDecisionRule<Joint<HistoryTree_p<number>>, number>> ddrs;

        // For all agents between N-1 and 1, we need to obtain their decision rules.
        for (int i = N - 2; i >= 0; i--)
        {
            // Accessible states for the a of agent I.
            std::vector<Joint<HistoryTree_p<number>>> acc_states_aI;
            // Actions for each of these acc_states_aI.
            std::vector<number> n_actions_aI;

            for(auto &x_o__p: sN)
            {
                Joint<action> partial_u_reversed;
                partial_u_reversed.push_back(greedy_uN);
                // Get actions of agents N-1 to I-1:
                for (int k = N - 2; k > i; k--)
                {   
                    // Initialize the partial joint history which will have all histories of agents between agent K and N.
                    Joint<HistoryTree_p<number>> partial_o;
                    // Get partial history for agent K, which will have histories of agents K to N:
                    for (int m = k; m < N; m++)
                    {   
                        std::cout << "m " << m << std::endl;
                        partial_o.push_back(x_o__p.first.second->at(m));
                    }
                    // Get precalculated decision rule of agent J.
                    DeterministicDecisionRule<Pair<Joint<HistoryTree_p<number>>, Joint<number>>, number> bK = b.at(k);

                    // Using bK and partial_o (which has all oM, for j<=m<=N) get uJ.
                    action uK = bK(std::make_pair(partial_o, partial_u_reversed));

                    // Add uK to partial_j_reversed
                    partial_u_reversed.push_back(uK);
                }
                // Initialize the partial joint history which will have all histories of agents between agent I and N.
                Joint<HistoryTree_p<number>> partial_o;
                for(agent j = i; j < N; j++)
                {   
                    partial_o.push_back(x_o__p.first.second->at(j));
                }
                // Add the input.
                acc_states_aI.push_back(partial_o);
                // Get precalculated decision rule of agent I.
                DeterministicDecisionRule<Pair<Joint<HistoryTree_p<number>>, Joint<number>>, number> bI = b.at(i);
                // Get action of agent I given all this.
                action uI = bI(std::make_pair(partial_o, partial_u_reversed));
                n_actions_aI.push_back(uI);
            }
            DeterministicDecisionRule<Joint<HistoryTree_p<observation>>, action> a1(acc_states_aI, n_actions_aI);
            ddrs.push_back(a1);
        }
        
        // Joint decision rule.
        HierarchicalPrivateJointDeterministicDecisionRule a(ddrs, greedy_uN);

        return a;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TValue HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getNextValueAt(const TState &state, number t){
        // Get next greedy action rule using next state.
        auto a = this->getGreedyAction(state, t);
        number N = this->env_->getUnderlyingProblem()->getNumAgents();
        Joint<action> u;
        u.push_back(a.second);
        for (int i = N - 2; i >= 0; i--)
        {
            Joint<HistoryTree_p<number>> o;
            for (int j = i; j < N; j++)
            {
                o.push_back(state.first.getLabel(state.second.at(j), j));
            }
            u.push_back(a.first.at(i)(o));
        }
        return this->getQValueAt(state, u, t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TValue HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::getNextValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, number t){
        return this->getNextValueAt(std::make_pair(private_occupancy_state, history), t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::updateQValueAt(const TState &state, const TAction &action, number t, TValue delta)
    {
        this->initializeIfNeeded(state);
        Joint<HistoryTree_p<number>> o = get_o(state.first, state.second);
        this->representation[state.first].updateQValueAt(o, action, t, delta);
        // auto os = this->representation.find(state.first);
        // if (os == this->representation.end()){
            
        // } else {
        //     this->representation[*os].updateQValueAt(o, action, t, delta);
        // }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::updateQValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, const TAction &action, number t, TValue delta)
    {   
        return this->updateQValueAt(std::make_pair(private_occupancy_state, history), action, t, delta);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::updateQValueAt(const TState &, const TAction &, number)
    {   
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::initializeIfNeeded(const TState &state)
    {   
        if (this->representation.find(state.first) == this->representation.end())
        {   
            this->representation.emplace(state.first, Container(0, learning_rate_, initializer_));
            // this->representation.at(state.first).initialize();
            this->representation.at(state.first).initialize(0);
        }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    Joint<HistoryTree_p<number>> HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::get_o(
        const typename TState::first_type &private_occupancy_state, 
        const typename TState::second_type &history
    ){
        // Number of agents.
        number N = this->env_->getUnderlyingProblem()->getNumAgents();
        // Construct the JointHistory.
        Joint<HistoryTree_p<number>> o;
        for (agent i = 0; i < N; i++)
        {
            o.push_back(private_occupancy_state.getLabel(history.at(i), i));
        }
        return o;
    }


    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    std::string HierarchicalTabularQValueFunction<TState, TAction, TValue, TMatrix>::str()
    {
        std::ostringstream res;
        res << "<hierarchical_tabular_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (auto it = this->representation.begin(); it != this->representation.end(); it++)
        {   
            if(it->second.representation[0].size() > 0){
                res << it->first << std::endl;
                res << it->second << std::endl;
            }
        }

        res << "</hierarchical_tabular_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm