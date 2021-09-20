#include <sdm/types.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <experimental/algorithm>

namespace sdm
{

    ExperienceMemory::ExperienceMemory(number horizon, int capacity)
    {
        this->experience_memory_ = std::vector<std::vector<sars_transition>>((horizon <= 0) ? 1 : horizon);
        this->capacity_ = capacity;
        this->position = 0;
    }

    void ExperienceMemory::push(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, const double reward, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<Action> &next_action, number t)
    {
        number real_time = (t >= this->experience_memory_.size()) ? 0 : t;
        if (this->experience_memory_[real_time].size() < this->capacity_)
        {
            this->experience_memory_[real_time].push_back(std::make_tuple(nullptr, nullptr, 0., nullptr, nullptr));
        }
        this->experience_memory_[real_time][this->position] = std::make_tuple(observation, action, reward, next_observation, next_action);
        this->position = (this->position + 1) % this->capacity_;
    }

    std::vector<ExperienceMemory::sars_transition> ExperienceMemory::sample(number t, int n)
    {
        std::vector<sars_transition> out;
        number real_time = (t >= this->experience_memory_.size()) ? 0 : t;
        std::experimental::sample(this->experience_memory_[real_time].begin(), this->experience_memory_[real_time].end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
        return out;
    }

    int ExperienceMemory::size()
    {
        return this->experience_memory_[0].size();
    }

} // namespace sdm