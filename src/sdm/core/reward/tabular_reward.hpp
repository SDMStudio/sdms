#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/reward/reward_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    /**
     * @brief This class provide a way to represent the reward model with a tabular representation.
     * 
     */
    class TabularReward : public RewardInterface
    {
    public:
        using value_type = double;
        // using vector_type = MappedVector<std::shared_ptr<State>, value_type>;
        using matrix_type = MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, value_type>;

        TabularReward();
        TabularReward(const TabularReward &copy);
        virtual ~TabularReward();

        void initReward(number num_states, number num_actions);

        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;
        void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &acction, double reward, number t = 0);

        double getMaxReward(number t = 0) const;
        double getMinReward(number t = 0) const;

        friend std::ostream &operator<<(std::ostream &os, const TabularReward &reward_fct)
        {
            os << "<reward min=\"" << reward_fct.min << "\" max=\"" << reward_fct.max << "\"> " << std::endl;
            os << reward_fct.rewards_.str() << std::endl;
            os << "</reward> " << std::endl;
            return os;
        }

    protected:
        //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
        // std::vector<vector_type> rewards_;
        matrix_type rewards_;

        double max = -std::numeric_limits<value_type>::max(), min = std::numeric_limits<value_type>::max();
    };

    // /**
    //  * @brief This class provides getter and setter methods for the reward model.
    //  */
    // class TemporalTabularReward : public RewardInterface,
    //                               public TemporalFunction<std::shared_ptr<TabularReward>>
    // {
    // public:
    //     using value_type = double;
    //     using vector_type = MappedVector<std::shared_ptr<State>, value_type>;

    //     TemporalTabularReward();
    //     TemporalTabularReward(const TemporalTabularReward &copy);

    //     void initReward(number, number);

    //     double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    //     {
    //         return this->getFunction(t)->getReward(state, action);
    //     }

    //     void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &acction, double reward, number t)
    //     {
    //         this->getFunction(t)->setReward(state, action, reward);
    //     }

    //     double getMaxReward(number t) const
    //     {
    //         return this->getFunction(t)->getMaxReward();
    //     }
    //     double getMinReward(number t) const
    //     {
    //         return this->getFunction(t)->getMinReward();
    //     }
    // };
}