// /*=============================================================================
//   Copyright (c) 2020 David Albert
// ==============================================================================*/
#include <sdm/world/interactive_world.hpp>
#include <sdm/parser/parser.hpp>
#include <tuple>

namespace sdm
{
    InteractiveWorld::InteractiveWorld(std::shared_ptr<POSG> intern_formalism) : GymInterface(std::make_shared<MultiDiscreteSpace>(intern_formalism->getObsSpace()), std::make_shared<MultiDiscreteSpace>(intern_formalism->getActionSpace())), internal_formalism_(intern_formalism)
    {
    }

    InteractiveWorld::InteractiveWorld(const POSG &intern_formalism) : GymInterface(std::make_shared<MultiDiscreteSpace>(intern_formalism.getObsSpace()), std::make_shared<MultiDiscreteSpace>(intern_formalism.getActionSpace())), internal_formalism_(std::make_shared<POSG>(intern_formalism))
    {
    }

    InteractiveWorld::InteractiveWorld(const std::string &filename) : InteractiveWorld(std::make_shared<POSG>(sdm::parser::parse_file(filename)))
    {
    }

    std::vector<number> InteractiveWorld::reset()
    {
        this->ctimestep_ = 0;
        number internal_state = this->internal_formalism_->init();
        // this->internal_formalism_->getDynamics();
    }

    std::tuple<std::vector<number>, std::vector<double>, bool> InteractiveWorld::step(std::vector<number> ja)
    {
        this->ctimestep_++;
        number jaction = this->internal_formalism_->getActionSpace().joint2single(ja);
        std::tuple<std::vector<double>, number, number> tuple_ =  this->internal_formalism_->getDynamicsGenerator(this->internal_formalism_->getInternalState(), jaction);

        this->internal_formalism_->setInternalState(std::get<2>(tuple_));

        std::vector<number> jobs = this->internal_formalism_->getObsSpace().single2joint(std::get<1>(tuple_));
        std::vector<double> rews = std::get<0>(tuple_);
        bool isFinished = (this->ctimestep_==this->internal_formalism_->getPlanningHorizon());

        return std::tie(jobs, rews, isFinished);
    }
} // namespace sdm
