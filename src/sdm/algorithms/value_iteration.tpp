
#include <sdm/algorithms/value_iteration.hpp>
namespace sdm
{
    template <typename TState, typename TAction>
    ValueIteration<TState,TAction>::ValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double error, int horizon) : problem_(problem), error_(error), horizon_(horizon)   
    {
    }

    /**
     * @brief Initialize the algorithm
     */
    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::do_initialize()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();

        policy_evaluation_1_ = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(this->problem_, this->horizon_,under_pb->getReward()->getMaxReward());
        policy_evaluation_2_ = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(this->problem_, this->horizon_,under_pb->getReward()->getMaxReward());

        policy_evaluation_1_->initialize();
        policy_evaluation_2_->initialize();
    }

    /**
     * @brief Solve a problem solvable by HSVI. 
     */
    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::do_solve()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();

        do
        {
            this->policy_evaluation_1_ = this->policy_evaluation_2_;

            for(int t = this->horizon_ -1; t>=0;t--)
            {
                for(const auto &state : under_pb->getStateSpace()->getAll() )
                {
                    this->policy_evaluation_2_->updateValueAt(state,t,this->policy_evaluation_1_->getQValueAt(state, t)->max());
                }
            }
        } while (this->borne());
        std::cout<<"Final value : "<<this->policy_evaluation_2_->str();
    }

    /**
     * @brief Test the learnt value function on one episode
     */
    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::do_test()
    {
        
    }
    
    template <typename TState, typename TAction>
    bool ValueIteration<TState,TAction>::borne()
    {
        double max_value = std::numeric_limits<double>::min();
        for(auto &state : this->policy_evaluation_1_->getSupport(0))
        {
            max_value = std::max(std::abs(policy_evaluation_1_->getValueAt(state,0) - policy_evaluation_2_->getValueAt(state,0)),max_value);
        }
        std::cout<<"\n error"<<this->error_;
        if (max_value<= this->error_)
        {
            return false;
        }else
        {
            return true;
        }
    }

    template <typename TState, typename TAction>
    std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> ValueIteration<TState,TAction>::getResult()
    {
        return this->policy_evaluation_2_;
    }

    template <typename TState, typename TAction>
    double ValueIteration<TState,TAction>::getResultOpti() 
    {
        return this->policy_evaluation_2_->getValueAt(this->problem_->getInitialState());
    }
}
