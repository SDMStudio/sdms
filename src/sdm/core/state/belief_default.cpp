#include <sdm/core/state/belief_default.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    BeliefDefault::BeliefDefault(){}

    BeliefDefault::BeliefDefault(double default_value)
    {
        this->map_nul = std::make_shared<MappedVector<std::shared_ptr<State>,double>>(default_value);
    }

    double BeliefDefault::getProbability(const std::shared_ptr<State> &) const
    {
        return this->getDefaultValue();
    }

    void BeliefDefault::setDefaultValue(double default_value)
    {
        this->map_nul->setDefault(default_value);
    }

    double BeliefDefault::getDefaultValue() const
    {
        return this->map_nul->getDefault();
    }

    std::vector<std::shared_ptr<State>> BeliefDefault::getStates() const
    {
        return this->map_nul->getIndexes();
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
        return this->getDefaultValue() == other->getDefaultValue() ;
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
        return this->map_nul;
    }


}