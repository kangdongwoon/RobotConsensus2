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
brl_msgs::brl_msgs Turtle_odom2;
geometry_msgs::Twist Turtle_twist[3];
static double des_position = 0.2;
static double err_position = 0;
static double preerr_position = 0;

void Robot2PoseSub(const brl_msgs::brl_msgs::ConstPtr &odom){
  Turtle_odom2.posx = odom->posx;
  Turtle_odom2.posy = odom->posy;
  Turtle_odom2.yaw = odom->yaw;
//  Turtle_odom2.pose.pose.orientation.y = odom->oriy;
//  Turtle_odom2.pose.pose.orientation.z = odom->oriz;
//  Turtle_odom2.pose.pose.orientation.w = odom->oriw;
  cout<<"2."<<"x.."<<odom->posx<<"y.."<<odom->posy<<endl;
}


namespace gazebo
{
  class ModelPush2 : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      RobotPose = nh.subscribe("/tb_2/odom",100,Robot2PoseSub);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ModelPush2::OnUpdate, this));
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      math::Pose cur_pose= this->model->GetWorldPose();
      Turtle_pose.Set(Turtle_odom2.posx,Turtle_odom2.posy,0,0,0,Turtle_odom2.yaw);


      math::Vector3 linVel, angVel;
      linVel.x = 0;
      linVel.y = 0;
      linVel.z = 0;
      angVel.x = 0;
      angVel.y = 0;
      angVel.z = 0;

      this->model->SetWorldPose(Turtle_pose);
      this->model->SetWorldTwist(linVel, angVel);
//      err_position = des_position - Turtle_odom2.pose.pose.position.x;

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
  GZ_REGISTER_MODEL_PLUGIN(ModelPush2)
}
