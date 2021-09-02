#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <ignition/math.hh>
#include <brl_msgs/brl_msgs.h>
using namespace std;

//Global Variables
brl_msgs::brl_msgs Turtle_odom1;
geometry_msgs::Twist Turtle_twist[3];
static double des_position = 0.2;
static double err_position = 0;
static double preerr_position = 0;

void Robot1PoseSub(const brl_msgs::brl_msgs::ConstPtr &odom){
  Turtle_odom1.posx = odom->posx;
  Turtle_odom1.posy = odom->posy;
  Turtle_odom1.yaw = odom->yaw;
}

namespace gazebo
{
  class ModelPush1 : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      RobotPose = nh.subscribe("/tb_1/odom",100,Robot1PoseSub);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ModelPush1::OnUpdate, this));
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      ignition::math::Pose3d cur_pose= this->model->WorldPose();

      Turtle_pose.Set(Turtle_odom1.posx,Turtle_odom1.posy,0,0,0,Turtle_odom1.yaw);

      ignition::math::Vector3d linVel, angVel;
      linVel.X(0);
      linVel.Y(0);
      linVel.Z(0);
      angVel.X(0);
      angVel.Y(0);
      angVel.Z(0);
      this->model->SetWorldPose(Turtle_pose);
      this->model->SetWorldTwist(linVel, angVel);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:
      ignition::math::Pose3d Turtle_pose;

    public:
      ros::NodeHandle nh;
      ros::Subscriber RobotPose;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush1)
}
