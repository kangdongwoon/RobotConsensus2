#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <brl_msgs/brl_msgs.h>
using namespace std;

//Global Variables
//nav_msgs::Odometry Turtle_odom1;
brl_msgs::brl_msgs Turtle_odom1;
geometry_msgs::Twist Turtle_twist[3];
static double des_position = 0.2;
static double err_position = 0;
static double preerr_position = 0;

void Robot1PoseSub(const brl_msgs::brl_msgs::ConstPtr &odom){
    Turtle_odom1.posx = odom->posx;
    Turtle_odom1.posy = odom->posy;
    Turtle_odom1.yaw = odom->yaw;
//  Turtle_odom1.pose.pose.position.x = odom->posx;
//  Turtle_odom1.pose.pose.position.y = odom->posy;
//  Turtle_odom1.pose.pose.orientation.x = odom->yaw;
//  Turtle_odom1.pose.pose.orientation.y = odom->oriy;
//  Turtle_odom1.pose.pose.orientation.z = odom->oriz;
//  Turtle_odom1.pose.pose.orientation.w = odom->oriw;
  cout<<"1."<<"x.."<<odom->posx<<"y.."<<odom->posy<<endl;
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
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      math::Pose cur_pose= this->model->GetWorldPose();
//      Turtle_pose.pos.x = Turtle_odom1.posx;
//      Turtle_pose.pos.y =-Turtle_odom1.posy;
//      Turtle_pose.pos.z = cur_pose.pos.z;
//      math::Vector3 rpy;
//      rpy.z = Turtle_odom1.yaw;
      Turtle_pose.Set(Turtle_odom1.posx,Turtle_odom1.posy,0,0,0,Turtle_odom1.yaw);
//      Turtle_pose.rot.x = Turtle_odom1.pose.pose.orientation.x;
//      Turtle_pose.rot.y = Turtle_odom1.pose.pose.orientation.y;
//      Turtle_pose.rot.z = Turtle_odom1.pose.pose.orientation.z;
//      Turtle_pose.rot.w = Turtle_odom1.pose.pose.orientation.w;

      math::Vector3 linVel, angVel;
      linVel.x = 0;
      linVel.y = 0;
      linVel.z = 0;
      angVel.x = 0;
      angVel.y = 0;
      angVel.z = 0;
      this->model->SetWorldPose(Turtle_pose);
      this->model->SetWorldTwist(linVel, angVel);
//      err_position = des_position - Turtle_odom1.pose.pose.position.x;

      //cout<<"err: "<<err_position<<endl;

//      Turtle_twist[2].linear.x = -1.0 * err_position; //- 0.6 *(err_position-preerr_position);
      //cout<<"lin_vel: "<<Turtle_twist[2].linear.x;
      //Robot2Pub.publish(Turtle_twist[2]);
      // Send cmd_vel 2 Real Turtlebot
//      preerr_position =err_position;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:
      math::Pose Turtle_pose;

    public:
      ros::NodeHandle nh;
//      ros::Subscriber Robot0Pose;
//      ros::Subscriber Robot1Pose;
//      ros::Subscriber Robot2Pose;
      ros::Subscriber RobotPose;
      //ros::Publisher Robot0Pub;
      //ros::Publisher Robot1Pub;
      //ros::Publisher Robot2Pub;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush1)
}
