#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  class ForcePlugin : public WorldPlugin
  {
    public: ForcePlugin() {}

    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      
      double force_start = 0.0

      if (_sdf->HasElement("robotNamespace"))
      {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
      }
      else
      {
        gzerr << "[force_plugin] Please specify a robotNamespace.\n";
      }

      // Create the node.
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(namespace_);

      std::string force_topic_ = "~/world_wind";

      // Subscribe to the topic, and register a callback
      this->sub = this->node.Subscribe(force_topic_,
        &ForcePlugin::OnUpdate, this);

      double pub_rate = 2.0;
      getSdfParam<double>(_sdf, "publishRate", pub_rate, pub_rate);
      pub_int_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
      getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);

      // force params
      double force_direction_ = 0.0;
      double force_velocity_ = 0.0;

      update_connection_ = event::Events::ConnectorWorldUpdateBegin(boost::bind(&ForcePlugin::OnUpdate, this, _1));
      force_pub = this->node->Advertise<physics_msgs::msgs::Wind>(force_topic_, 10);

      // this->SetDirection(force_direction_);
      // this->SetVelocity(force_velocity_);
    }

    // public: void SetVelocity()
    // {

    // }

    // private: void OnMsg(ConstVector3dPtr &_msg)
    // {
      
    // }

    public: void ForcePlugin::OnUpdate(const common::UpdateInfo& _info)
    {
    #if GAZEBO_MAJOR_VERSION >= 9
      common::Time now = this->world->SimTime();
    #else
      common::Time now = this->world->GetSimTime();
    #endif

      if ((now - last_time_).Double() < pub_int_ || pub_int_ == 0.0)
      {
        return;
      }
      last_time_ = now;

      // Calculate force
      gazebo::msgs::Vector3d force_direction;
      force_direction.X() = force_velocity_*cos(force_direction_);
      force_direction.Y() = force_velocity_*sin(force_direction_);
      force_direction.Z() = 0;

      force_msg.set_frame_id()

    }


  }
}