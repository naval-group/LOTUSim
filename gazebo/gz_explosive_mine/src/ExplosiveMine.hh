#ifndef SYSTEM_PLUGIN_EXPLOSIVEMINE_HH_
#define SYSTEM_PLUGIN_EXPLOSIVEMINE_HH_

#include <gz/sim/System.hh>

namespace mine
{
    class ExplosiveMinePrivate;

    class ExplosiveMine : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate
    {
    public:
        ExplosiveMine();
        /// \brief Destructor
    public:
        ~ExplosiveMine() override = default;

        // Documentation inherited
    public:
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        /// Documentation inherited
    public:
        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) final;

        // Documentation inherited
    public:
        void PostUpdate(
            const gz::sim::UpdateInfo &_info,
            const gz::sim::EntityComponentManager &_ecm) override;

        /// \brief Private data pointer
    private:
        std::unique_ptr<ExplosiveMinePrivate> dataPtr;
    };
}

#endif // EXPLOSIVEMINE_LIBRARY_H
