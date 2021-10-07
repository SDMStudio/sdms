#pragma once

#include <torch/torch.h>
#include <smd/utils/nn/networks.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     *  Q(S,A, \theta) = \sum_{o,u} S(o) * A(u|o) * q(S, o,u; \theta) -- nous cherchons a implemente q(; \theta)
     */
    class DeepQValueFunction : public QValueFunction
    {
    protected:

        std::vector<Mlpnet> qnetworks;

        number nactions, nobservations, memory, horizon;
        
    public:

        DeepQValueFunction(number num_actions, number num_observations, number memory, number horizon)
        {
            number input, output;
            for(auto step=0; step<horizon; ++step)
            {
                ouput = num_actions;
                input = 2*std::pow(num_observations, std::min(memory, step));   
                this->qnetworks[step] = Mlpnet(input, output);      
            }

            this->initialize();
        }

        /**
         * @brief Initialize the value function 
         */
        void initialize();

        double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0){
            // definir le tenseur torch 
            
        }

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta);

        void updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, PieceWiseLinearConvexQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    };

} // namespace sdm