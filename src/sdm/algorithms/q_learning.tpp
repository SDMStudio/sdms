#include <sdm/algorithms/q_learning.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TObservation>
    QLearning<TState, TAction, TObservation>::QLearning(number trials, std::string results)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    QLearning<TState, TAction, TObservation>::QLearning(std::shared_ptr<GymInterface> env,
                                          std::shared_ptr<QValueFunction<TState, TAction>> lower_bound,
                                          std::shared_ptr<QValueFunction<TState, TAction>> upper_bound,
                                          number planning_horizon,
                                          double epsilon,
                                          number num_max_trials = 10000)
    {
    }

   

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::do_episode(std::shared_ptr<GymInterface> env, long nb_timesteps, number planning_horizon = 100, number test_freq = 1000, number save_freq = 1000, std::string save_folder = "models", bool render = false, double time_laps = 0., number verbose = 1, long timestep_init = 0, std::string log_file = None)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    void QLearning<TState, TAction, TObservation>::select_action(TObservation obs)
    {
    }

} // namespace sdm
