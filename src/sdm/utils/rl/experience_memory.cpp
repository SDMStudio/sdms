#include <sdm/utils/rl/experience_memory.hpp>

namespace sdm
{

    ExperienceMemory::ExperienceMemory(number horizon, int capacity)
    {
        for (number t = 0; t < horizon; t++)
        {
            experience_memory_.push_back({});
        }
        this->capacity_ = capacity;
        this->position = 0;
    }

    void ExperienceMemory::push(const std::shared_ptr<Observation>& observation, const std::shared_ptr<Action>& action, const double reward, const std::shared_ptr<Observation>& next_observation, const std::shared_ptr<Action>& next_action, number t)
    {
        if (this->experience_memory_[t].size() < this->capacity_)
        {
            this->experience_memory_[t].push_back(std::make_tuple(nullptr, nullptr, 0., nullptr, nullptr));
        }
        this->experience_memory_[t][this->position] = std::make_tuple(observation, action, reward, next_observation, next_action);
        this->position = (this->position + 1) % this->capacity_;
    }

    std::vector<ExperienceMemory::sars_transition> ExperienceMemory::sample(number t, int n)
    {
        std::vector<sars_transition> out;
        std::experimental::sample(this->experience_memory_[t].begin(), this->experience_memory_[t].end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
        return out;
    }

    int ExperienceMemory::size()
    {
        return this->experience_memory_[0].size();
    }

} // namespace sdm