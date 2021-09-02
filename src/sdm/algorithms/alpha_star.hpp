#pragma once

#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <chrono>
#include <string>

namespace sdm
{

  /**
   * @brief The class for the algorithm [A*](http://ai.stanford.edu/users/nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf).
   */
  class AlphaStar : public Algorithm, public std::enable_shared_from_this<AlphaStar>
  {
  protected:

    /**
     * @brief The state in terms of A* algorithm.
     * 
     */
    class AlphaStarItem : public State
    {
    public :
        double value_f_, value_g_;
        int horizon_;
        std::shared_ptr<State> current_element;

        AlphaStarItem(const std::shared_ptr<State>& element, double value_g,double value_f, int horizon) : value_f_(value_f), value_g_(value_g), horizon_(horizon), current_element(element)
        {}
        
        bool operator<(std::shared_ptr<AlphaStarItem> const & b)
        {
          return (this->value_f_ == b->value_f_) ? this->value_g_ > b->value_g_ : this->value_f_ < b->value_f_;
        }

        std::string str() const
        {
          std::ostringstream res;
          res << "AlphaStarState[" << this->current_element->str();
          res <<", G_value "<<this->value_g_;
          res <<", F_value "<<this->value_f_;
          res <<", horizon "<<this->horizon_<<" ]";
          return res.str();
        }
    };

    /** @brief The problem to be solved */
    std::shared_ptr<SolvableByHSVI> world_;

    /** @brief The representation. */
    std::shared_ptr<TabularValueFunction> bound_;

    /** @brief The logger */
    std::shared_ptr<MultiLogger> logger_;

    /** @brief Some hyperparameters for the algorithm */
    number planning_horizon_;
    std::string name_ = "backward_induction";

    std::shared_ptr<State> start_state;
    std::vector<std::shared_ptr<AlphaStarItem>> openSet;
    std::vector<std::shared_ptr<AlphaStarItem>> FSet;

    std::vector<std::unordered_map<std::shared_ptr<State>,std::shared_ptr<AlphaStarItem>>> map_element_to_alpha_item;


    std::chrono::high_resolution_clock::time_point start_time, current_time;
    double duration;

  public:
    /**
     * @brief Construct the AlphaStar algorithm with custom paramters.
     * 
     * @param world the problem to be solved by A*
     * @param name the name of the algorithm (this name is used to save logs)
     */
    AlphaStar(std::shared_ptr<SolvableByHSVI> &world,
              std::string name = "A*");

    std::shared_ptr<AlphaStar> getptr();

    /**
     * @brief Initialize the algorithm.
     * 
     */
    void do_initialize();

    /**
     * @brief Solve a problem with A* algorithm. 
     */
    void do_solve();

    /**
     * @brief Test the learnt value function on one episode
     */
    void do_test();

    /**
     * @brief Save the lower bound under "name_lb.bin"
     * 
     */
    void do_save();

    /**
     * @brief Check the end of HSVI algo
     * 
     * @param s the current state
     * @param h the current timestep
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false elsewhere
     */
    bool do_stop(const std::shared_ptr<State> &, double /*cost_so_far*/, number);

    /**
     * @brief Explore a state.
     * 
     * @param s the state to explore
     * @param h the timestep of the exploration
     */
    void do_explore(const std::shared_ptr<State> &s, double /*cost_so_far*/, number h);

    /**
     * @brief Get the bound value function 
     */
    std::shared_ptr<ValueFunction> getBound() const;

    int getTrial(){return 0;}

    double getResult();

    void saveResults(std::string filename, double other);

    void updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information);

    void initLogger();
    
    /**
     * @brief Compare two A* items.
     * 
     * @param item_1 the first item
     * @param item_2 the second item
     */
    static bool compare(const std::shared_ptr<AlphaStarItem>& item_1, const std::shared_ptr<AlphaStarItem>& item_2) 
    {
      return item_1->operator<(item_2);
    }

    static double TIME_TO_REMOVE;

  };
} // namespace sdm
