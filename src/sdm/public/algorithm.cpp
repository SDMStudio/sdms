#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/public/algorithm.hpp>

namespace sdm
{
    Algorithm::Algorithm(std::string name) : name(name) {}

    Algorithm::~Algorithm() {}

    std::string Algorithm::getName() const
    {
        return name;
    }

    void Algorithm::startExecutionTime()
    {
        start_execution_time = std::chrono::high_resolution_clock::now();
    }

    void Algorithm::stopExecutionTime()
    {
        stop_execution_time = std::chrono::high_resolution_clock::now();
    }

    std::chrono::high_resolution_clock::time_point Algorithm::getStartExectionTime() const
    {
        return start_execution_time;
    }

    std::chrono::high_resolution_clock::time_point Algorithm::getStopExectionTime() const
    {
        return stop_execution_time;
    }

    double Algorithm::getExecutionTime() const
    {
        return std::Performance::computeTime(start_execution_time);
    }

    void Algorithm::saveResults(std::string filename, std::string format)
    {
    }

    void Algorithm::printInfo()
    {
        std::cout << config::LOG_SDMS << "-------------------" << std::endl;
        std::cout << config::LOG_SDMS << "# name=" << name << std::endl;
        std::cout << config::LOG_SDMS << "-------------------" << std::endl;
    }

}