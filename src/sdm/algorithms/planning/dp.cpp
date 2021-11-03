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

    void DynamicProgramming::initLogger()
    {
        // ************* Global Logger ****************
        std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lower_bound({}) \t Size_upper_bound({}) \t Time({})  \n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that prints logs in a file
        // auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_lower_bound", "Size_upper_bound", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
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