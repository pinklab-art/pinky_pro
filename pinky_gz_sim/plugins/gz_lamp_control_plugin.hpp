#ifndef SYSTEM_PLUGIN_GZ_LAMP_CONTROL_HH_
#define SYSTEM_PLUGIN_GZ_LAMP_CONTROL_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Material.hh>

namespace pinky_gz_sim
{
    class LampControlPlugin:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
        public: void Configure(const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventMgr) override;

        public: void PreUpdate(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;


        private:
            gz::sim::Model model_;
            gz::transport::Node node_;
            double last_update_time_;

            double g_val_;
            double g_dir_;
    };
}

#endif //SYSTEM_PLUGIN_GZ_LAMP_CONTROL_HH_