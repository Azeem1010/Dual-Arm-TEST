#include "dual_arm_gripper/policy.hpp"
#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

namespace dual_arm_gripper {
class SuctionSystem final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  struct Arm {
    gz::sim::Entity parent{gz::sim::kNullEntity};
    gz::sim::Entity joint{gz::sim::kNullEntity};
    std::optional<Command> pending;
    std::string held;
    std::uint64_t last_sequence{};
    gz::math::Vector3d tip;
    gz::transport::Node::Publisher status;
  };
  gz::sim::Entity model_{gz::sim::kNullEntity};
  std::array<Arm,2> arms_;
  std::mutex mutex_;
  gz::transport::Node transport_;
  void reply(Arm &arm,std::uint64_t sequence,const std::string &state) {
    gz::msgs::StringMsg message;
    message.set_data(std::to_string(sequence)+","+state);
    arm.status.Publish(message);
  }
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    model_=entity;
    for (std::size_t i=0;i<arms_.size();++i) {
      const auto name="arm"+std::to_string(i+1);
      arms_[i].tip=sdf->Get<gz::math::Vector3d>(name+"_tip");
      arms_[i].status=transport_.Advertise<gz::msgs::StringMsg>("/gripper/"+name+"/state");
      transport_.Subscribe<gz::msgs::StringMsg>("/gripper/"+name+"/command",[this,i](const gz::msgs::StringMsg &message) {
        auto command=parse(message.data());
        std::scoped_lock lock(mutex_);
        if (!command || command->sequence<=arms_[i].last_sequence || arms_[i].pending) return;
        arms_[i].pending=std::move(command);
      });
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    std::scoped_lock lock(mutex_);
    for (std::size_t i=0;i<arms_.size();++i) {
      auto &arm=arms_[i];
      if (!arm.pending) continue;
      const auto command=*arm.pending;
      arm.pending.reset();
      arm.last_sequence=command.sequence;
      if (!command.attach) {
        if (arm.joint!=gz::sim::kNullEntity) ecm.RequestRemoveEntity(arm.joint);
        arm.joint=gz::sim::kNullEntity;
        arm.held.clear();
        reply(arm,command.sequence,"released");
        continue;
      }
      if (!arm.held.empty() || (!arms_[1-i].held.empty() && arms_[1-i].held==command.object)) {
        reply(arm,command.sequence,"fault:already_attached"); continue;
      }
      arm.parent=gz::sim::Model(model_).LinkByName(ecm,"arm"+std::to_string(i+1)+"_cad_link_4");
      const auto child_model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(command.object));
      const auto child=gz::sim::Model(child_model).LinkByName(ecm,"body");
      if (arm.parent==gz::sim::kNullEntity || child==gz::sim::kNullEntity) {
        reply(arm,command.sequence,"fault:missing_entity"); continue;
      }
      const auto parent_pose=gz::sim::worldPose(arm.parent,ecm);
      const auto cap_pose=gz::sim::worldPose(child,ecm);
      const auto tip=parent_pose.Pos()+parent_pose.Rot().RotateVector(arm.tip);
      const auto top=cap_pose.Pos()+cap_pose.Rot().RotateVector(gz::math::Vector3d(0,0,.005));
      const auto delta=cap_pose.Rot().RotateVectorReverse(tip-top);
      const auto suction_direction=parent_pose.Rot().RotateVector(gz::math::Vector3d(0,1,0));
      const auto cap_normal=cap_pose.Rot().RotateVector(gz::math::Vector3d(0,0,1));
      if (!contact_ok(std::hypot(delta.X(),delta.Y()),delta.Z(),-suction_direction.Dot(cap_normal))) {
        reply(arm,command.sequence,"fault:no_contact"); continue;
      }
      arm.joint=ecm.CreateEntity();
      ecm.CreateComponent(arm.joint,gz::sim::components::DetachableJoint({arm.parent,child,"fixed"}));
      arm.held=command.object;
      reply(arm,command.sequence,"attached:"+arm.held);
    }
  }
};
}
GZ_ADD_PLUGIN(dual_arm_gripper::SuctionSystem,gz::sim::System,
              dual_arm_gripper::SuctionSystem::ISystemConfigure,
              dual_arm_gripper::SuctionSystem::ISystemPreUpdate)
