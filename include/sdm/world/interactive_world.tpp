#include <sdm/world/interactive_world.hpp>
#include <sdm/parser/parser.hpp>
#include <tuple>

namespace sdm
{
    template <typename TDecProcess>
    InteractiveWorld<TDecProcess>::InteractiveWorld(std::shared_ptr<TDecProcess> intern_formalism) : GymInterface<observation_space_type, action_space_type>(std::make_shared<TDecProcess>(intern_formalism->getObsSpace()), std::make_shared<TDecProcess>(intern_formalism->getActionSpace())), internal_formalism_(intern_formalism)
    {
    }

    template <typename TDecProcess>
    InteractiveWorld<TDecProcess>::InteractiveWorld(const TDecProcess &intern_formalism) : GymInterface<observation_space_type, action_space_type>(std::make_shared<TDecProcess>(intern_formalism.getObsSpace()), std::make_shared<TDecProcess>(intern_formalism.getActionSpace())), internal_formalism_(std::make_shared<TDecProcess>(intern_formalism))
    {
    }

    template <typename TDecProcess>
    InteractiveWorld<TDecProcess>::InteractiveWorld(const std::string &filename) : InteractiveWorld(std::make_shared<TDecProcess>(sdm::parser::parse_file(filename)))
    {
    }

    template <typename TDecProcess>
    typename InteractiveWorld<TDecProcess>::observation_type InteractiveWorld<TDecProcess>::reset()
    {
        this->ctimestep_ = 0;
        number internal_state = this->internal_formalism_->init();
        // this->internal_formalism_->getDynamics();
    }

    template <typename TDecProcess>
    std::tuple<typename InteractiveWorld<TDecProcess>::observation_type, std::vector<double>, bool> InteractiveWorld<TDecProcess>::step(InteractiveWorld<TDecProcess>::action_type ja)
    {
        // this->ctimestep_++;
        // number jaction = this->internal_formalism_->getActionSpace().joint2single(ja);
        // std::tuple<std::vector<double>, number, number> tuple_ = this->internal_formalism_->getDynamicsGenerator(this->internal_formalism_->getInternalState(), jaction);

        // this->internal_formalism_->setInternalState(std::get<2>(tuple_));

        // std::vector<number> jobs = this->internal_formalism_->getObsSpace().single2joint(std::get<1>(tuple_));
        // std::vector<double> rews = std::get<0>(tuple_);
        // bool isFinished = (this->ctimestep_ == this->internal_formalism_->getPlanningHorizon());

        // return std::tie(jobs, rews, isFinished);
    }
} // namespace sdm
