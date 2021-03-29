#pragma once

#include <sdm/utils/value_function/initializer.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    class MDPInitializer : public Initializer<TState, TAction>
    {
    public:
        std::string algo_name_;
        double error_, trials_;

    public:
        MDPInitializer(std::string algo_name, double error = 0.01, int trials = 10000);
        void init(ValueFunction<TState, TAction> *vf);
    };
} // namespace sdm
#include <sdm/utils/value_function/initializer/mdp_initializer.tpp>
