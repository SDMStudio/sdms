#include <sdm/utils/value_function/prunable_structure.hpp>

namespace sdm
{

    PrunableStructure::PrunableStructure(number horizon, int freq_pruning) : horizon(horizon), freq_pruning(freq_pruning)
    {
    }

    void PrunableStructure::doPruning(number trial)
    {
        if (trial  % this->getPruningFrequency() == 0)
        {
            for (number time = 0; time < this->horizon; time++)
            {
                this->prune(time);
            }
        }
    }

    int PrunableStructure::getPruningFrequency() const
    {
        return this->freq_pruning;
    }

} // namespace sdm
