#pragma once

#include <random>
#include <math.h>
#include <iterator>
#include <utility>
#include <sdm/utils/rl/experience_memory_interface.hpp>

namespace sdm
{

    class ExperienceMemory : public ExperienceMemoryInterface
    {
    protected:
        number capacity_;
        std::vector<number> positions_;
        std::vector<std::vector<sars_transition>> experience_memory_;

    public:
        ExperienceMemory(number horizon, int capacity = 1);

        void push(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const double reward, const std::shared_ptr<State> &next_state, const std::shared_ptr<Action> &next_action, number t);

        std::vector<sars_transition> sample(number t, int n = 1);

        int size();

        int getCapacity() const;

    };
} // namespace sdm