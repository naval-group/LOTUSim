#include "Scheduler.hpp"

// Function to convert a map to a string
template <typename K, typename V>
std::string mapToString(const std::map<K, V> &m)
{
    std::ostringstream oss;
    oss << "{";
    for (auto it = m.begin(); it != m.end(); ++it) {
        if (it != m.begin())
            oss << ", ";
        oss << it->first << ": " << it->second;
    }
    oss << "}";
    return oss.str();
}

std::string
mapToString2(const std::map<std::string, std::vector<std::string>> &map)
{
    std::ostringstream oss;
    oss << "{";
    for (auto it = map.begin(); it != map.end(); ++it) {
        if (it != map.begin())
            oss << ", ";
        oss << "\"" << it->first << "\": [";
        for (size_t i = 0; i < it->second.size(); ++i) {
            if (i > 0)
                oss << ", ";
            oss << "\"" << it->second[i] << "\"";
        }
        oss << "]";
    }
    oss << "}";
    return oss.str();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    SchedulerSingleton *sched = SchedulerSingleton::GetInstance();

    sched->AddComponentManager("Test1");
    sched->AddComponentManager("Test2");
    sched->AddComponentManager("Test3");

    auto service_list = sched->get_service_names_and_types();
    RCLCPP_INFO(sched->get_logger(), mapToString2(service_list).c_str());

    auto callback_grp =
        rclcpp::CallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);

    // while (rclcpp::ok()) {
    //     RCLCPP_INFO(manager1->get_logger(), "Spinning everything");
    //     sched->ChangeExecutorsOrder(RandomisedType::RANDOM);
    //     for (std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
    //              executor : sched->GetExecutors()) {
    //         executor->spin_once();
    //     }
    // }

    std::random_device rd;
    std::default_random_engine rng(rd());

    auto managers = sched->GetManagers();
    auto executor = sched->GetExecutor();

    while (rclcpp::ok()) {
        for (std::shared_ptr<rclcpp_components::ComponentManager> manager :
             managers) {
            executor->remove_node(manager->get_node_base_interface());
        }
        std::shuffle(managers.begin(), managers.end(), rng);
        for (std::shared_ptr<rclcpp_components::ComponentManager> manager :
             managers) {
            executor->add_node(manager->get_node_base_interface());
        }
        executor->spin_once(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}