#include <sdm/exception.hpp>

namespace sdm
{
    // ******************
    // Value Initializer
    // ****************** 
    ValueInitializer::ValueInitializer(double v) : value(v){}

    void ValueInitializer::initBase(std::shared_ptr<BaseValueFunction> vf)
    {
        if (vf->getHorizon() < 1)
        {
            vf->initialize(this->value);
        }
        else
        {
            for (number t = 0; t < vf->getHorizon(); t++)
            {
                vf->initialize(this->value, t);
            }

            vf->initialize(0, vf->getHorizon());
        }
    }

    void ValueInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        this->initBase(vf);
    }

    void ValueInitializer::init(std::shared_ptr<QValueFunction> vf)
    {
        this->initBase(vf);
    }

    // ******************
    // Zero Initializer
    // ****************** 
    ZeroInitializer::ZeroInitializer() : ValueInitializer(0){}

    // ******************
    // Bound Initializer
    // ****************** 
    BoundInitializer::BoundInitializer() {}
    BoundInitializer::BoundInitializer(std::shared_ptr<SolvableByHSVI> world,double value) : value_(value), world_(world) {}

    void BoundInitializer::init(std::shared_ptr<ValueFunction> vf)
    {

        if (vf->isInfiniteHorizon())
        {
            vf->initialize(this->computeValueInfiniteHorizon(vf));
        }
        else
        {
            double tot = 0;
            vf->initialize(tot, vf->getHorizon());
            for (number t = vf->getHorizon(); t > 0; t--)
            {
                tot = this->getValue(vf, t) + this->world_->getUnderlyingProblem()->getDiscount(t) * tot;
                vf->initialize(tot, t - 1);
            }
        }
    }

    double BoundInitializer::getValue(std::shared_ptr<ValueFunction>, number t)
    {
        return (this->callback_value == nullptr) ? this->value_ : ((*world_->getUnderlyingProblem()).*callback_value)(t);
    }

    double BoundInitializer::computeValueInfiniteHorizon(std::shared_ptr<ValueFunction> )
    {
        // auto under_pb = this->world_->getUnderlyingProblem();

        // long l = log(1 - this->discount_) * this->error_ / this->reward_->getMaxReward();
        // number t = 0;
        // double value = this->getValue(vf, t), factor = 1.;
        // do
        // {
        //     factor *= under_pb->getDiscount(t);
        //     value += factor * this->getValue(vf, t + 1);
        //     t++;
        // } while (factor < 1.e-10);
        // value = floor(value) + (value > 0); // value = -2.99 --> floor(-2.99) + 0 = -3.0 and 2.99 --> floor(2.99) + 1 = 2 + 1 = 3.0
        // return value;
        throw exception::NotImplementedException();
    }

    // ******************
    // MinI Initializer
    // ****************** 
    MinInitializer::MinInitializer(std::shared_ptr<SolvableByHSVI> world)
    {
        this->world_ = world;
        std::cout << "In MinInitalizer" << std::endl; 
    }

    void MinInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // this->world_->getUnderlyingProblem()->getMinReward();
        this->callback_value = &MDPInterface::getMinReward;
        BoundInitializer::init(vf);
    }

    // ******************
    // Max Initializer
    // ****************** 
    MaxInitializer::MaxInitializer(std::shared_ptr<SolvableByHSVI> world)
    {
        this->world_= world;
        std::cout << "In MaxInitalizer" << std::endl;
    }

    void MaxInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // this->value_ = this->world_->getUnderlyingProblem()->getMaxReward(t);
        this->callback_value = &MDPInterface::getMaxReward;
        BoundInitializer::init(vf);
    }

    // ******************
    // Blind Initializer
    // ****************** 
    BlindInitializer::BlindInitializer(std::shared_ptr<SolvableByHSVI> world) 
    {
        this->world_ = world;
        std::cout << "In BlindInitalizer" << std::endl;
    }

    void BlindInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        auto under_pb = this->world_->getUnderlyingProblem();
        std::vector<double> ra, rt;

        for (number t = 0; t < vf->getHorizon(); t++)
        {
            ra.clear();
            for (auto &a : *under_pb->getActionSpace(t))
            {
                ra.push_back(std::numeric_limits<double>::max());
                for (auto &s : *under_pb->getStateSpace(t))
                {
                    ra.back() = std::min(under_pb->getReward(std::static_pointer_cast<State>(s), std::static_pointer_cast<Action>(a), t), ra.back());
                }
            }
            rt.push_back(*std::max_element(ra.begin(), ra.end()));
        }

        this->value_ = *std::min_element(rt.begin(), rt.end());
        BoundInitializer::init(vf);
    }

} // namespace sdm
