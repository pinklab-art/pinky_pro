#include <string>
#include <gz/common/Console.hh>
#include <gz/common/Material.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/material_color.pb.h>

#include "gz_lamp_control_plugin.hpp"

namespace pinky_gz_sim
{

void LampControlPlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager & /*_ecm*/,
    gz::sim::EventManager & /*_eventMgr*/)
{
    model_ = gz::sim::Model(_entity);
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    sdf::ElementPtr argument_sdf = sdfPtr->GetElement("link");
    while (argument_sdf) {
        std::string lamp_name = argument_sdf->Get<std::string>();
        gzmsg << lamp_name << std::endl;
        argument_sdf = argument_sdf->GetNextElement("link");
    }

    g_val_ = 0.0;
    g_dir_ = 0.0;
    last_update_time_ = 0.0;

    gzmsg << "Configure!!!" << std::endl;
}

void LampControlPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager & /*_ecm*/)
{
    gz::msgs::MaterialColor materialColorMsgFirst;
    materialColorMsgFirst.mutable_entity()->set_name("robot_lamp_visual");
    materialColorMsgFirst.set_entity_match(gz::msgs::MaterialColor::EntityMatch::MaterialColor_EntityMatch_FIRST);
    gz::msgs::Set(materialColorMsgFirst.mutable_diffuse(), gz::math::Color(0.0f, g_val_, 0.0f, 1.0f));
    gz::msgs::Set(materialColorMsgFirst.mutable_ambient(), gz::math::Color(0.0f, g_val_, 0.0f, 1.0f));
    gz::msgs::Set(materialColorMsgFirst.mutable_emissive(), gz::math::Color(0.0f, g_val_, 0.0f, 1.0f));

    double currentTime = std::chrono::duration<double>(_info.simTime).count();
    if (currentTime - last_update_time_ >= 0.1)
    {
        if(g_dir_ == 0)
        {
            g_val_ += 0.1;
            if(g_val_ >= 1.0) {
                g_dir_ = 1;
                g_val_ = 1.0;
            }
        }
        else
        {
            g_val_ -= 0.1;
            if(g_val_ <= 0.0) {
                g_dir_ = 0;
                g_val_ = 0;
            }
        }
        last_update_time_ = currentTime;

        auto pub = node_.Advertise<gz::msgs::MaterialColor>("/world/default/material_color");
        pub.Publish(materialColorMsgFirst);
    }
}

}

GZ_ADD_PLUGIN(
    pinky_gz_sim::LampControlPlugin,
    gz::sim::System,
    pinky_gz_sim::LampControlPlugin::ISystemConfigure,
    pinky_gz_sim::LampControlPlugin::ISystemPreUpdate)


