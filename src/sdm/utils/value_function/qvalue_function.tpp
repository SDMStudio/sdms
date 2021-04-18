#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    QValueFunction<TState, TAction, TValue>::QValueFunction()
    {
    }

    template <typename TState, typename TAction, typename TValue>
    QValueFunction<TState, TAction, TValue>::QValueFunction(number horizon) : BaseValueFunction<TState, TAction, TValue>(horizon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TValue QValueFunction<TState, TAction, TValue>::getValueAt(const TState &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    TAction QValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, number t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

    template <typename TState, typename TAction, typename TValue>
    TAction QValueFunction<TState, TAction, TValue>::getBestAction_his(std::shared_ptr<GymInterface<TState, TAction>> env, const TState &state, number t)
    {   
        PrivateOccupancyState<number, JointHistoryTree_p<number>> s2 = state.first;
        // JointHistoryTree_p<number> o = state.second;
        HistoryTree_p<number> o2 = state.second->getIndividualHistory(1);
        std::vector<TState> acc_states;
        std::vector<number> n_actions;
        for (auto &o1: s2.getIndividualHistories(1)){
            for (number u2; u2 < env->getUnderlyingProblem()->getNumActions(1); u2++){
                Joint<typename TAction::first_type::value_type::output_type> actions;
                actions.push_back(u2);
                std::vector<HistoryTree_p<number>> histories = {o1, o2};
                JointHistoryTree_p<number> o = std::make_shared<typename TState::second_type>(JointHistoryTree(histories));
                TState acc_state = std::make_tuple(o, actions);
                acc_states.push_back(acc_state);
                auto qvalues = this->getQValueAt(acc_state, t);
                for(number i; env->getUnderlyingProblem()->getNumActions(); i++){
                    auto accs = env->getUnderlyingProblem()->getActionSpace().single2joint(i);
                    if (accs[1] != u2){
                        qvalues.at[i] = std::numeric_limits<double>::lowest();
                    }
                }
                number best_u = qvalues->argmax();
                number best_u1 = env->getUnderlyingProblem()->getActionSpace().single2joint(best_u)[0];
                n_actions.push_back(best_u1);
            }
        }
        DeterministicDecisionRule<Pair<HistoryTree_p<number>, Joint<number>>, number> a1(acc_states, n_actions);
        
        // this is just to get a vector, cuz idk how to initialize it
        auto qvalues = this->getQValueAt(state, t);
        for(number i; env->getUnderlyingProblem()->getNumActions(); i++){
            qvalues.at[i] = std::numeric_limits<double>::lowest();
        }
        // ^ and this too
        for(number u2; u2 < env->getUnderlyingProblem()->getNumActions(0); u2++){
            qvalues.at[u2] = 0;
            Joint<typename TAction::first_type::value_type::output_type> actions;
            Joint<typename TAction::first_type::value_type::output_type> jactions;
            actions.push_back(u2);
            // o only has o1 btw, not o2
            for(auto &x_o__prob: s2){
                HistoryTree_p<number> o1 = x_o__prob.first.second->getIndividualHistory(0);
                std::vector<HistoryTree_p<number>> histories = {o1, o2};
                JointHistoryTree_p<number> o = std::make_shared<typename TState::second_type>(JointHistoryTree(histories));
                TState acc_state = std::make_tuple(o, actions);
                number u1 = a1(acc_state);
                jactions.push_back(u1);
                jactions.push_back(u2);
                number u = env->getUnderlyingProblem()->getActionSpace().joint2single(jactions);
                qvalues.at[u2] += x_o__prob.second * this->getQValueAt(acc_state, u, t);
            }
        }
        number best_u2 = qvalues->argmax();
        

        Joint<DeterministicDecisionRule<Pair<HistoryTree_p<number>, Joint<number>>, number>> jddr;
        jddr.push_back(a1);
        // Pair<Joint<DeterministicDecisionRule<Pair<HistoryTree_p<number>, Joint<number>>, number>>, number>> oaction = std::make_pair(jddr, best_u2);
        
        return std::make_pair(jddr, best_u2);

        // auto qvalues = this->getQValueAt(state, t);
        // return qvalues->argmax();
    }



} // namespace sdm