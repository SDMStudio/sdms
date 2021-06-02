#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{
    /**
     * @brief This class provides getter and setter methods for the reward model.
     */
    class TabularReward : public BaseReward
    {
    public:
        using value_type = double;
        using vector_type = DenseVector<std::shared_ptr<State>, value_type>;

        TabularReward();
        TabularReward(number, number);
        TabularReward(const TabularReward &copy);

        void initReward(number, number);

        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        void setReward(const std::shared_ptr<State>, const std::shared_ptr<Action>, number t, double);

        double getMaxReward(number t) const;
        double getMinReward(number t) const;

        friend std::ostream &operator<<(std::ostream &os, const Reward &reward_fct)
        {
            number ja;
            os << "<reward> " << std::endl;
            for (ja = 0; ja < reward_fct.rewards.size(); ++ja)
            {
                os << "\t" << reward_fct.getReward(ja) << "" << std::endl;
            }
            os << "</reward> " << std::endl;
            return os;
        }

    protected:
        //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
        std::vector<vector_type> rewards;

        double max = -999999, min = +999999;
    };

}