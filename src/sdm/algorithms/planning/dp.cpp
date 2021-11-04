#include <sdm/algorithms/planning/dp.hpp>

namespace sdm
{
    DynamicProgramming::DynamicProgramming(std::shared_ptr<SolvableByHSVI> world, double error, std::string name)
        : Algorithm(name), world(world), error(error)
    {
    }

    void DynamicProgramming::printStartInfo()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "START PLANNING (" << this->getAlgorithmName() << ")" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;
    }

    void DynamicProgramming::printEndInfo()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "END PLANNING (" << this->getAlgorithmName() << ")" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;
    }

    std::shared_ptr<SolvableByHSVI> DynamicProgramming::getWorld() const
    {
        return world;
    }

    number DynamicProgramming::getTrial()
    {
        return trial;
    }
}