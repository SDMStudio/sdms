#pragma once

#include <vector>
#include <tuple>
#include <random>
#include <math.h>
#include <vector>
#include <iterator>
#include <experimental/algorithm>
#include <tuple>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/any.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
    //! \brief Transition of observation, action, reward, next_observation
    typedef std::tuple<std::shared_ptr<Observation>, std::shared_ptr<Action>, double, std::shared_ptr<Observation>> sars_transition;

    class ExperienceMemory
    {
    protected:
        int capacity_;
        std::vector<std::vector<sars_transition>> experience_memory_;
        int position;

    public:
        ExperienceMemory(number horizon, int capacity = 1)
        {
            for (number t = 0; t < horizon; t++)
            {
                experience_memory_.push_back({});
            }
            this->capacity_ = capacity;
            this->position = 0;
        }

        void push(const std::shared_ptr<Observation>& observation, const std::shared_ptr<Action>& action, const double reward, const std::shared_ptr<Observation>& next_observation, number t)
        {
            if (this->experience_memory_[t].size() < this->capacity_)
            {
                this->experience_memory_[t].push_back(std::make_tuple(nullptr, nullptr, 0., nullptr));
            }
            this->experience_memory_[t][this->position] = std::make_tuple(observation, action, reward, next_observation);
            this->position = (this->position + 1) % this->capacity_;
        }

        std::vector<sars_transition> sample(number t, int n = 1)
        {
            std::vector<sars_transition> out;
            std::experimental::sample(this->experience_memory_[t].begin(), this->experience_memory_[t].end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
            return out;
        }

        int size()
        {
            return this->experience_memory_[0].size();
        }
    };
} // namespace sdm