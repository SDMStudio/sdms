#include <sdm/core/state/belief_default.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    BeliefDefault::BeliefDefault(){}

    BeliefDefault::BeliefDefault(double default_value) : default_value_(default_value) {}

    double BeliefDefault::getProbability(const std::shared_ptr<State> &) const
    {
        return this->default_value_;
    }

    void BeliefDefault::setDefaultValue(double default_value)
    {
        this->default_value_ = default_value;
    }

    double BeliefDefault::getDefaultValue() const
    {
        return this->default_value_;
    }

    std::vector<std::shared_ptr<State>> BeliefDefault::getStates() const
    {
        return std::vector<std::shared_ptr<State>>();
    }

    size_t BeliefDefault::size() const
    {
        return 0;
    }

    void BeliefDefault::setProbability(const std::shared_ptr<State> &, double )
    {
        throw sdm::exception::Exception("It isn't possible de add state in the class Default Belief");

    }

    void BeliefDefault::addProbability(const std::shared_ptr<State> &, double )
    {
        throw sdm::exception::Exception("It isn't possible de add state in the class Default Belief");
    }

    bool BeliefDefault::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->default_value_ == other->getDefaultValue() ;
    }

    double BeliefDefault::operator^(const std::shared_ptr<BeliefInterface> &other) const
    {
        return this->getDefaultValue() * other->getDefaultValue();
    }

    double BeliefDefault::norm_1() const
    {
        return 0;
    }

    std::string BeliefDefault::str() const
    {
        std::ostringstream res;
        res << "BeliefDefault[ default :"<<this->getDefaultValue()<<"]";
        return res.str();
    }

    std::shared_ptr<VectorInterface<std::shared_ptr<State>,double>> BeliefDefault::getVectorInferface()
    {
        throw sdm::exception::NotImplementedException();
    }


}