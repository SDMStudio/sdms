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

#include <sdm/types.hpp>

namespace sdm
{
    //! \brief Transition of state, action, reward, next_state
    typedef std::tuple<state, action, double, state> sars_transition;

    class ReplayMemory
    {
    protected:
        int capacity;
        std::vector<sars_transition> memory;
        int position;

    public:
        ReplayMemory(int capacity)
        {
            this->capacity = capacity;
            this->position = 0;
        }

        void push(sars_transition t)
        {
            if (this->memory.size() < this->capacity)
            {
                this->memory.push_back(std::make_tuple(0, 0, 0., 0));
            }
            this->memory[this->position] = t;
            this->position = (this->position + 1) % this->capacity;
        }

        std::vector<sars_transition> sample(int n)
        {
            std::vector<sars_transition> out;
            std::experimental::sample(this->memory.begin(), this->memory.end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
            return out;
        }

        int size()
        {
            return this->memory.size();
        }
    };
} // namespace sdm