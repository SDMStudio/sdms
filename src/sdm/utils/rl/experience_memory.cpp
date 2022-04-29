#include <sdm/types.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <experimental/algorithm>

namespace sdm
{

    ExperienceMemory::ExperienceMemory(number horizon, int capacity)
    {
        this->experience_memory_ = std::vector<std::vector<sars_transition>>((horizon == 0) ? 1 : horizon);
        this->positions_ = std::vector<number>((horizon == 0) ? 1 : horizon, 0);
        this->capacity_ = capacity;
    }

    void ExperienceMemory::push(const std::shared_ptr<State> &observation, const std::shared_ptr<Action> &action, const double reward, const std::shared_ptr<State> &next_observation, const std::shared_ptr<Action> &next_action, number step)
    {
        assert(step < this->experience_memory_.size());

        if (this->experience_memory_[step].size() < capacity_)
        {
            this->experience_memory_[step].push_back(std::make_tuple(observation, action, reward, next_observation, next_action));
        }
        else
        {
            this->experience_memory_[step][this->positions_[step]] = std::make_tuple(observation, action, reward, next_observation, next_action);
        }
        this->positions_[step] = (this->positions_[step] + 1) % this->capacity_;
    }

    std::vector<ExperienceMemory::sars_transition> ExperienceMemory::sample(number step, int n)
    {
        assert(step < this->experience_memory_.size());

        std::vector<sars_transition> out;
        std::sample(this->experience_memory_[step].begin(), this->experience_memory_[step].end(), std::back_inserter(out), n, std::mt19937{std::random_device{}()});
        return out;
    }

    int ExperienceMemory::size()
    {
        return this->experience_memory_[0].size();
    }

    int ExperienceMemory::getCapacity() const
    {
        return this->capacity_;
    }

} // namespace sdm