#include <sdm/algorithms/alpha_star.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

namespace sdm
{
    double AlphaStar::TIME_TO_REMOVE = 0;

    AlphaStar::AlphaStar(std::shared_ptr<SolvableByHSVI> &world,
                         std::string name) : DynamicProgramming(world, 0, name)
    {
        auto tabular_backup = std::make_shared<TabularBackup>(world);
        auto action_tabular = std::make_shared<ExhaustiveActionSelection>(world);

        auto init = std::make_shared<POMDPInitializer>(world, "HSVI");

        this->bound_ = std::make_shared<TabularValueFunction>(world_->getUnderlyingProblem()->getHorizon(), init, tabular_backup, action_tabular, false);
    }

    std::shared_ptr<AlphaStar> AlphaStar::getptr()
    {
        return this->shared_from_this();
    }

    void AlphaStar::initLogger()
    {

        // ************* Global Logger ****************
        std::string format = "#> Point at Horizon :\t{}\t, current value :\t{} \t, max estimation \t{}\t Size Bound to explore\t{} \t Total Size \t{} \t Time({})  \n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Horizon", "G_value", "F_value", "Size_explore", "Size_total", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void AlphaStar::initialize()
    {
        this->bound_->initialize();
        this->initLogger();

        for (size_t i = 0; i <= this->world_->getUnderlyingProblem()->getHorizon(); i++)
        {
            this->map_element_to_alpha_item.push_back(std::unordered_map<std::shared_ptr<State>, std::shared_ptr<AlphaStarItem>>());
        }
    }

    void AlphaStar::solve()
    {
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start AlphaStar \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        this->start_state = this->world_->getInitialState();

        this->map_element_to_alpha_item.at(0).emplace(this->start_state, std::make_shared<AlphaStarItem>(this->start_state, 0, -this->bound_->evaluate(this->start_state, 0).second, 0));
        this->openSet.push_back(this->map_element_to_alpha_item.at(0).at(this->start_state));

        this->start_time = std::chrono::high_resolution_clock::now();
        this->duration = 0.0;

        while (this->openSet.size() != 0 && !this->stop(*this->openSet.begin(), 0, this->openSet.begin()->get()->horizon_))
        {
            this->current_time = std::chrono::high_resolution_clock::now();
            this->logger_->log(this->openSet.begin()->get()->horizon_, this->openSet.begin()->get()->value_g_, this->openSet.begin()->get()->value_f_, this->openSet.size(), std::static_pointer_cast<ValueFunction>(this->bound_)->getSize(), this->duration - AlphaStar::TIME_TO_REMOVE);
            this->updateTime(current_time, "Time_to_remove");

            this->explore(*this->openSet.begin(), 0, this->openSet.begin()->get()->horizon_);

            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();
        }

        // Logging (save data and print algorithms variables)
        //---------------------------------//
        this->current_time = std::chrono::high_resolution_clock::now();
        this->logger_->log(this->map_element_to_alpha_item.at(0).at(this->start_state)->horizon_, this->map_element_to_alpha_item.at(0).at(this->start_state)->value_g_, this->map_element_to_alpha_item.at(0).at(this->start_state)->value_f_, this->openSet.size(), std::static_pointer_cast<ValueFunction>(this->bound_)->getSize(), this->duration - AlphaStar::TIME_TO_REMOVE);
        this->updateTime(current_time, "Time_to_remove");
        //---------------------------------//

        std::cout << "Best value " << this->bound_->getRepresentation(this->world_->getUnderlyingProblem()->getHorizon()).max() << std::endl;
    }

    bool AlphaStar::stop(const std::shared_ptr<State> &, double, number h)
    {
        return this->world_->getUnderlyingProblem()->getHorizon() <= h || this->openSet.size() == 0;
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

                if (h != this->world_->getUnderlyingProblem()->getHorizon())
                {
                    auto action_space = this->world_->getActionSpaceAt(state, h);

                    double cost;
                    for (const auto &action : *action_space)
                    {
                        auto observation_space = this->world_->getObservationSpaceAt(state, action->toAction(), h);

                        cost = this->world_->getDiscount(h) * this->world_->getReward(state, action->toAction(), h);

                        for (const auto &observation : *observation_space)
                        {
                            auto [next_state, proba] = this->world_->getNextState(this->bound_, state, action->toAction(), observation->toObservation(), h);

                            this->map_element_to_alpha_item.at(h + 1).emplace(next_state, std::make_shared<AlphaStarItem>(next_state, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), h + 1));

                            auto next_alpha_star_element = this->map_element_to_alpha_item.at(h + 1).at(next_state);

                            if (next_alpha_star_element->value_g_ > alpha_star_element->value_g_ - cost)
                            {
                                next_alpha_star_element->value_g_ = alpha_star_element->value_g_ - cost;
                                next_alpha_star_element->value_f_ = next_alpha_star_element->value_g_ - this->bound_->evaluate(next_state, h + 1).second;
                                this->bound_->setValueAt(next_state, -next_alpha_star_element->value_g_, h + 1);

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

    std::shared_ptr<ValueFunction> AlphaStar::getBound() const
    {
        return this->bound_;
    }

    void AlphaStar::updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information)
    {
        auto time = std::Performance::computeTime(start_time);

        if (information == "Time_to_remove")
        {
            AlphaStar::TIME_TO_REMOVE += time;
        }
    }

} // namespace sdm