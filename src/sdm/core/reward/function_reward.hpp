#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/reward/reward_model.hpp>

namespace sdm
{
    /**
     * @brief This class provides getters and setters for the reward model.
     */
    class FunctionReward : public RewardModel
    {
    public:
        FunctionReward();
        FunctionReward(const std::string &expression);
        FunctionReward(const FunctionReward &copy);

        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const;

        double getMaxReward(number agent_id, number t) const;
        double getMinReward(number agent_id, number t) const;

        void setMaxReward(number t, double max_value);
        void setMinReward(number t, double min_value);

    protected:
        std::string expression;
        double max = -999999, min = +999999;
    };
}