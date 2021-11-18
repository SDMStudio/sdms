#include <vector>
#include <sdm/types.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/world/gym/robot_bin/robot_bin.hpp>

namespace sdm
{
    namespace gym
    {
        RobotBin::RobotBin(int size_x, int size_y)
            : size_x_(size_x),
              size_y_(size_y)
        {

            std::vector<std::shared_ptr<CoordState>> list_possible_coord;
            for (int x = 0; x < size_x; x++)
            {
                for (int y = 0; y < size_y; y++)
                {
                    auto coord = std::make_shared<CoordState>(Coordinate(x, y));

                    list_possible_coord.push_back(coord);
                }
            }

            // Init the coordinate space
            this->coord_space_ = std::make_shared<DiscreteSpace>(list_possible_coord);
            // Init the state space
            this->state_space_ = std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{this->coord_space_, this->coord_space_});
            // Init the coordinates of the robot
            this->coord_robot_ = this->coord_space_->getItem(0)->to<CoordState>();

            // Reset the coordinates of the waste
            this->coord_garbage_ = this->coord_space_->getItem(this->getSizeX()*this->getSizeY()-1)->to<CoordState>();

            std::vector<std::shared_ptr<Action>> list_actions{
                std::make_shared<DiscreteActionString>("left"),
                std::make_shared<DiscreteActionString>("up"),
                std::make_shared<DiscreteActionString>("right"),
                std::make_shared<DiscreteActionString>("down")};
            this->action_space_ = std::make_shared<DiscreteSpace>(list_actions);
        }

        std::shared_ptr<Space> RobotBin::getActionSpaceAt(const std::shared_ptr<State> &, number)
        {
            return this->action_space_;
        }

        std::shared_ptr<Action> RobotBin::getRandomAction(const std::shared_ptr<State> &, number)
        {
            return this->action_space_->sample()->toAction();
        }

        std::shared_ptr<State> RobotBin::reset()
        {
            // Reset the coordinates of the robot
            this->coord_robot_ = this->coord_space_->getItem(0)->to<CoordState>();

            return this->getJointCoordinateState(this->coord_robot_, this->coord_garbage_);
        }

        int RobotBin::getSizeX() const
        {
            return this->size_x_;
        }

        int RobotBin::getSizeY() const
        {
            return this->size_y_;
        }

        std::shared_ptr<RobotBin::CoordState> RobotBin::getCoordinate(int x, int y, int add_to_x, int add_to_y)
        {
            number index_new_coord = this->coord_space_->getItemIndex(this->coord_robot_) + add_to_y + add_to_x * this->getSizeY();
            return this->coord_space_->getItem(index_new_coord)->to<CoordState>();
        }

        std::shared_ptr<State> RobotBin::getJointCoordinateState(const std::shared_ptr<CoordState> &coord_robot, const std::shared_ptr<CoordState> &coord_garbage)
        {
            number N = this->coord_space_->getNumItems(),
                   INDEX_JOINT_COORD = N * this->coord_space_->getItemIndex(coord_robot) + this->coord_space_->getItemIndex(coord_garbage);
            auto state = this->state_space_->getItem(INDEX_JOINT_COORD);
            return state->toState();
        }

        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> RobotBin::step(std::shared_ptr<Action> action)
        {
            auto str_action = std::dynamic_pointer_cast<DiscreteActionString>(action)->getAction();
            bool is_done = true;
            double reward = 0;

            if ((str_action == "up") && (this->coord_robot_->getState().getX() > 0))
            {
                this->coord_robot_ = this->getCoordinate(this->coord_robot_->getState().getX(),
                                                         this->coord_robot_->getState().getY(), -1, 0);
            }
            else if ((str_action == "down") && (this->coord_robot_->getState().getX() < this->getSizeX() - 1))
            {
                this->coord_robot_ = this->getCoordinate(this->coord_robot_->getState().getX(),
                                                         this->coord_robot_->getState().getY(), 1, 0);
            }
            else if ((str_action == "left") && (this->coord_robot_->getState().getY() > 0))
            {
                this->coord_robot_ = this->getCoordinate(this->coord_robot_->getState().getX(),
                                                         this->coord_robot_->getState().getY(), 0, -1);
            }
            else if ((str_action == "right") && (this->coord_robot_->getState().getY() < this->getSizeY() - 1))
            {
                this->coord_robot_ = this->getCoordinate(this->coord_robot_->getState().getX(),
                                                         this->coord_robot_->getState().getY(), 0, 1);
            }

            // Check if robot found the waste
            if (this->coord_robot_ != this->coord_garbage_)
            {
                reward = -1;
                is_done = false;
            }

            return {this->getJointCoordinateState(this->coord_robot_, this->coord_garbage_), {reward}, is_done};
        }

    } // namespace sdm
}