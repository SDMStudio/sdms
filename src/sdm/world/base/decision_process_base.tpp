#include <sdm/world/base/decision_process_base.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::DecisionProcessBase() {}

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::DecisionProcessBase(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp)
        : StochasticProcessBase<TStateSpace, TDistrib>(state_sp),
          action_space_(action_sp)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::DecisionProcessBase(std::shared_ptr<TStateSpace> state_sp,
                                                                                  std::shared_ptr<TActionSpace> action_sp,
                                                                                  TDistrib start_distrib,
                                                                                  number planning_horizon,
                                                                                  double discount,
                                                                                  Criterion criterion)
        : StochasticProcessBase<TStateSpace, TDistrib>(state_sp, start_distrib),
          action_space_(action_sp),
          planning_horizon_(planning_horizon),
          discount_(discount),
          criterion_(criterion)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    std::string DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getFileName()
    {
        return this->filename_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setFileName(std::string filename)
    {
        this->filename_ = filename;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    Criterion DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getCriterion()
    {
        return this->criterion_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setCriterion(Criterion criterion)
    {
        this->criterion_ = criterion;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    double DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getBound()
    {
        return this->bound_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setBound(double bound)
    {
        this->bound_ = std::min(1.0 / (bound * (1.0 - this->discount)), 1.0);
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    double DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getDiscount()
    {
        return this->discount_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setDiscount(double discount)
    {
        this->discount_ = discount;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    number DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getPlanningHorizon()
    {
        return this->planning_horizon_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setPlanningHorizon(number planning_horizon)
    {
        this->planning_horizon_ = planning_horizon;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    std::shared_ptr<TActionSpace> DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getActionSpace() const
    {
        return this->action_space_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    void DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::setActionSpace(std::shared_ptr<TActionSpace> action_space)
    {
        this->action_space_ = action_space;
    }

} // namespace sdm