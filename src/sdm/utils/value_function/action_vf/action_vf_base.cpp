#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
namespace sdm
{
    ActionVFBase::ActionVFBase(){}

    ActionVFBase::ActionVFBase(const std::shared_ptr<SolvableByHSVI>& world): world_(world){}

    ActionVFBase::~ActionVFBase(){}
}