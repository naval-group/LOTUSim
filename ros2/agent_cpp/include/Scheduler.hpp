#pragma once
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <chrono>
#include <composition_interfaces/srv/load_node.hpp>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <vector>

enum class RandomisedType { NONE, RANDOM };
enum class SchedulerType { EXECUTOR_IN_THREADS, RANDOMISED_EXECUTOR };

/**
 * The Singleton class defines the `GetInstance` method that serves as an
 * alternative to constructor and lets clients access the same instance of this
 * class over and over.
 */
class SchedulerSingleton : public rclcpp::Node {

    /**
     * The Singleton's constructor/destructor should always be private to
     * prevent direct construction/desctruction calls with the `new`/`delete`
     * operator.
     */

private:
    static SchedulerSingleton *pinstance_;
    static std::mutex mutex_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> st_executor_;
    std::vector<std::shared_ptr<rclcpp_components::ComponentManager>> managers_;
    std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>>
        executors_;
    SchedulerType scheduler_type_;

protected:
    SchedulerSingleton()
        : rclcpp::Node("scheduler_node")
    {
        scheduler_type_ = SchedulerType::RANDOMISED_EXECUTOR;
        st_executor_ =
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    ~SchedulerSingleton() {}

public:
    /**
     * Singletons should not be cloneable.
     */
    SchedulerSingleton(SchedulerSingleton &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const SchedulerSingleton &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */

    static SchedulerSingleton *GetInstance();
    /**
     * Finally, any singleton should define some business logic, which can be
     * executed on its instance.
     */
    void AddExecutor(
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor)
    {
        executors_.push_back(executor);
        RCLCPP_INFO(
            get_logger(),
            "The scheduler now contains %d executors",
            executors_.size());
    }

    void AddComponentManager(std::string manager_name)
    {
        auto manager = std::make_shared<rclcpp_components::ComponentManager>(
            st_executor_, manager_name);

        st_executor_->add_node(manager);
        managers_.push_back(manager);

        RCLCPP_INFO(
            get_logger(),
            "The executor now contains %d managers",
            managers_.size());
    }

    std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>>
    GetExecutors() const
    {
        return executors_;
    }

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
    GetExecutor() const
    {
        return st_executor_;
    }

    std::vector<std::shared_ptr<rclcpp_components::ComponentManager>>
    GetManagers() const
    {
        return managers_;
    }

    SchedulerType GetSchedulerType() const { return scheduler_type_; }

    void ChangeExecutorsOrder(RandomisedType type)
    {
        switch (type) {
        case (RandomisedType::RANDOM): {
            std::random_device rd;
            std::default_random_engine rng(rd());
            std::shuffle(executors_.begin(), executors_.end(), rng);
        }
        default: {
        }
        }
    }
};

/**
 * Static methods should be defined outside the class.
 */

SchedulerSingleton *SchedulerSingleton::pinstance_{nullptr};
std::mutex SchedulerSingleton::mutex_;

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. RU:
 */
SchedulerSingleton *SchedulerSingleton::GetInstance()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr) {
        pinstance_ = new SchedulerSingleton();
    }
    return pinstance_;
}