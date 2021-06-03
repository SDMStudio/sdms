
namespace sdm
{
    template <typename T>
    Joint<T>::Joint() : BaseJoint<T>() {}

    template <typename T>
    Joint<T>::Joint(const std::vector<T> &joint_item) : BaseJoint<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(const std::vector<number> &, const std::vector<T> &joint_item) : BaseJoint<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(std::initializer_list<T> list_values) : BaseJoint<T>(list_values) {}



    // Specialisation for the Joint Action 
    Joint<std::shared_ptr<Action>>::Joint() : BaseJoint<std::shared_ptr<Action>>() {std::cout<<"\n Joint Action";}

    Joint<std::shared_ptr<Action>>::Joint(const std::vector<std::shared_ptr<Action>> &joint_item) : BaseJoint<std::shared_ptr<Action>>(joint_item) {}

    Joint<std::shared_ptr<Action>>::Joint(const std::vector<number> &, const std::vector<std::shared_ptr<Action>> &joint_item) : BaseJoint<std::shared_ptr<Action>>(joint_item) {}

    Joint<std::shared_ptr<Action>>::Joint(std::initializer_list<std::shared_ptr<Action>> list_values) : BaseJoint<std::shared_ptr<Action>>(list_values) {}



    // Specialisation for the Joint State 
    Joint<std::shared_ptr<State>>::Joint() : BaseJoint<std::shared_ptr<State>>() {std::cout<<"\n Joint State";}

    Joint<std::shared_ptr<State>>::Joint(const std::vector<std::shared_ptr<State>> &joint_item) : BaseJoint<std::shared_ptr<State>>(joint_item) {}

    Joint<std::shared_ptr<State>>::Joint(const std::vector<number> &, const std::vector<std::shared_ptr<State>> &joint_item) : BaseJoint<std::shared_ptr<State>>(joint_item) {}

    Joint<std::shared_ptr<State>>::Joint(std::initializer_list<std::shared_ptr<State>> list_values) : BaseJoint<std::shared_ptr<State>>(list_values) {}




    // Specialisation for the Joint Observation  
    Joint<std::shared_ptr<Observation>>::Joint() : BaseJoint<std::shared_ptr<Observation>>() {std::cout<<"\n Joint Obervation";}

    Joint<std::shared_ptr<Observation>>::Joint(const std::vector<std::shared_ptr<Observation>> &joint_item) : BaseJoint<std::shared_ptr<Observation>>(joint_item) {}

    Joint<std::shared_ptr<Observation>>::Joint(const std::vector<number> &, const std::vector<std::shared_ptr<Observation>> &joint_item) : BaseJoint<std::shared_ptr<Observation>>(joint_item) {}

    Joint<std::shared_ptr<Observation>>::Joint(std::initializer_list<std::shared_ptr<Observation>> list_values) : BaseJoint<std::shared_ptr<Observation>>(list_values) {}

} // namespace sdm
