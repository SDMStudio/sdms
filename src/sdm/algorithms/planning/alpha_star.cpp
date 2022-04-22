#include <sdm/algorithms/planning/alpha_star.hpp>

#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

namespace sdm
{
    double AlphaStar::TIME_TO_REMOVE = 0;

    AlphaStar::AlphaStar(const std::shared_ptr<SolvableByHSVI> &world,
                         const std::shared_ptr<ValueFunction> &value_function,
                         std::string name) : DynamicProgramming(world, 0, name)
    {
        if (auto derived = std::dynamic_pointer_cast<TabularValueFunction>(value_function))
        {
            this->value_function = derived;
        }
        else
        {
            throw sdm::exception::TypeError("A* does not yet support other representation than the tabular one.");
        }
    }

    std::shared_ptr<AlphaStar> AlphaStar::getptr()
    {
        return this->shared_from_this();
    }

    void AlphaStar::initLogger()
    {

        // ************* Global Logger ****************
        std::string format = config::LOG_SDMS + "Horizon {:<6} GValue {:<12.4f} FValue (MaxEstim) {:<12.4f} SizeExplore {:<10} Size {:<10} Time {:<12.4f}\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Horizon", "G_value", "F_value", "Size_explore", "D", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void AlphaStar::initialize()
    {
        getBound()->initialize();

        this->initLogger();

        for (size_t i = 0; i <= getWorld()->getHorizon(); i++)
        {
            this->map_element_to_alpha_item.push_back(std::unordered_map<std::shared_ptr<State>, std::shared_ptr<AlphaStarItem>>());
        }
    }

    void AlphaStar::solve()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "START A*" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;

        this->start_state = getWorld()->getInitialState();

        this->map_element_to_alpha_item.at(0).emplace(this->start_state, std::make_shared<AlphaStarItem>(this->start_state, 0, -getBound()->getValueAt(this->start_state, 0), 0));
        this->openSet.push_back(this->map_element_to_alpha_item.at(0).at(this->start_state));

        this->start_time = std::chrono::high_resolution_clock::now();
        this->duration = 0.0;

        while (this->openSet.size() != 0 && !this->stop(*this->openSet.begin(), 0, this->openSet.begin()->get()->horizon_))
        {
            this->current_time = std::chrono::high_resolution_clock::now();
            this->logger_->log(this->openSet.begin()->get()->horizon_, this->openSet.begin()->get()->value_g_, this->openSet.begin()->get()->value_f_, this->openSet.size(), std::dynamic_pointer_cast<ValueFunction>(getBound())->getSize(), this->duration - AlphaStar::TIME_TO_REMOVE);
            this->updateTime(current_time, "Time_to_remove");

            this->explore(*this->openSet.begin(), 0, this->openSet.begin()->get()->horizon_);

            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();
        }

        // Logging (save data and print algorithms variables)
        //---------------------------------//
        this->current_time = std::chrono::high_resolution_clock::now();
        this->logger_->log(this->map_element_to_alpha_item.at(0).at(this->start_state)->horizon_, this->map_element_to_alpha_item.at(0).at(this->start_state)->value_g_, this->map_element_to_alpha_item.at(0).at(this->start_state)->value_f_, this->openSet.size(), std::dynamic_pointer_cast<ValueFunction>(getBound())->getSize(), this->duration - AlphaStar::TIME_TO_REMOVE);
        this->updateTime(current_time, "Time_to_remove");
        //---------------------------------//

        std::cout << config::LOG_SDMS << "FINALE VALUE : " << getBound()->getRepresentation(getWorld()->getHorizon()).max() << std::endl;

        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "END A*" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << config::NO_COLOR << std::endl;
    }

    bool AlphaStar::stop(const std::shared_ptr<State> &, double, number h)
    {
        return getWorld()->getHorizon() <= h || this->openSet.size() == 0;
    }

    void AlphaStar::explore(const std::shared_ptr<State> &state_tmp, double cost_so_far, number h)
    {
        try
        {
            if (!this->stop(state_tmp, cost_so_far, h))
            {
                auto alpha_star_element = std::static_pointer_cast<AlphaStarItem>(state_tmp);
                auto state = alpha_star_element->current_element;

                this->openSet.erase(this->openSet.begin());
                // Boucle sur tous les successeurs possibles

                if (h != getWorld()->getHorizon())
                {
                    auto action_space = getWorld()->getActionSpaceAt(state, h);

                    double cost;
                    for (const auto &action : *action_space)
                    {
                        auto observation_space = getWorld()->getObservationSpaceAt(state, action, h);

                        cost = getWorld()->getDiscount(h) * getWorld()->getReward(state, action, h);

                        for (const auto &observation : *observation_space)
                        {
                            auto [next_state, proba] = getWorld()->getNextState(state, action, observation, h);

                            this->map_element_to_alpha_item.at(h + 1).emplace(next_state, std::make_shared<AlphaStarItem>(next_state, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), h + 1));

                            auto next_alpha_star_element = this->map_element_to_alpha_item.at(h + 1).at(next_state);

                            if (next_alpha_star_element->value_g_ > alpha_star_element->value_g_ - cost)
                            {
                                next_alpha_star_element->value_g_ = alpha_star_element->value_g_ - cost;
                                next_alpha_star_element->value_f_ = next_alpha_star_element->value_g_ - getBound()->getValueAt(next_state, h + 1);
                                getBound()->setValueAt(next_state, -next_alpha_star_element->value_g_, h + 1);

                                if (std::find(this->openSet.begin(), this->openSet.end(), next_alpha_star_element) == this->openSet.end())
                                {
                                    this->openSet.push_back(next_alpha_star_element);
                                    std::sort(std::begin(this->openSet), std::end(this->openSet), compare);
                                }
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "AlphaStar::explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void AlphaStar::test()
    {
    }

    void AlphaStar::save()
    {
    }

    std::shared_ptr<TabularValueFunction> AlphaStar::getBound() const
    {
        return this->value_function;
    }

    void AlphaStar::updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information)
    {
        auto time = std::Performance::computeTime(start_time);

        if (information == "Time_to_remove")
        {
            AlphaStar::TIME_TO_REMOVE += time;
        }
    }

    std::string AlphaStar::getAlgorithmName()
    {
        return "A*";
    }

} // namespace sdm