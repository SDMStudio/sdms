#include <sdm/utils/rl/deep_experience_memory.hpp>

namespace sdm
{

    DeepExperienceMemory::DeepExperienceMemory(int capacity)
    {
        this->capacity_ = capacity;
        this->position = 0;
    }

    void DeepExperienceMemory::push(const std::shared_ptr<Observation>& observation, const std::shared_ptr<Action>& action, const double reward, const std::shared_ptr<Observation>& next_observation, const std::shared_ptr<Action>& next_action, number t)
    {
        if (this->experience_memory_.size() < this->capacity_)
        {
            this->experience_memory_.push_back(std::make_tuple(0, nullptr, nullptr, 0., nullptr, nullptr));
        }
        this->experience_memory_[this->position] = std::make_tuple(t, observation, action, reward, next_observation, next_action);
        this->position = (this->position + 1) % this->capacity_;
    }

    std::vector<DeepExperienceMemory::sars_transition> DeepExperienceMemory::sample(int n)
    {
        std::vector<sars_transition> out;
        std::experimental::sample(this->experience_memory_.begin(), this->experience_memory_.end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
        return out;
    }

    std::vector<DeepExperienceMemory::sars_transition> DeepExperienceMemory::sample(number t, int n)
    {
        return this->sample(n);
    }

    int DeepExperienceMemory::size()
    {
        return this->experience_memory_.size();
    }

} // namespace sdm