#include <functional>
#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
    class SeawolfControl : public ModelPlugin
    {
    public:

        void RCCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) 
        {
            //excepts a 6 member 1D float array
            if(msg->data.size() != 6) 
            {
                return;    
            }

            pitchRate = msg->data.at(0);
            rollRate = msg->data.at(1);
            verticalRate = msg->data.at(2);
            yawRate = msg->data.at(3);
            forwardRate = msg->data.at(4);
            strafeRate = msg->data.at(5);
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _parent;

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SeawolfControl::onUpdate, this));

            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ros::NodeHandle n;

            //The control code needs to know our altitude and heading so create publishers to send that data
            altPub = n.advertise<std_msgs::Float64>("wolf_gazebo/global_alt", 100);
            hdgPub = n.advertise<std_msgs::Float64>("wolf_gazebo/compass_hdg", 100);

            //we need to know what RC commands the control code is sending out so we subscribe to that topic
            RCsub = n.subscribe<std_msgs::Float32MultiArray>("wolf_RC_output", 100, &SeawolfControl::RCCallback, this);
        }

    public:
        void onUpdate()
        {
            //we need to get our position information to the controller so the control loops can spin
            auto pose = this->model->WorldPose();

            std_msgs::Float64 alt;
            std_msgs::Float64 hdg;
            alt.data = pose.Pos().Z();
            hdg.data = (pose.Rot().Yaw() * 180.0)/M_PI;

            altPub.publish(alt);
            hdgPub.publish(hdg);


            //we should move the model according to the RC rates
            this->model->SetLinearVel(pose.Rot().RotateVector(ignition::math::Vector3d(forwardRate, strafeRate, verticalRate)));
            this->model->SetAngularVel(ignition::math::Vector3d(pitchRate, rollRate, yawRate));
        }

        
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        ros::Publisher altPub;
        ros::Publisher hdgPub;
        ros::Subscriber RCsub;

        //rate of movement
        float forwardRate = 0.0f;
        float strafeRate = 0.0f;
        float verticalRate = 0.0f;
        
        //rate of rotation
        float pitchRate = 0.0f;
        float rollRate = 0.0f;
        float yawRate = 0.0f;
    };

    GZ_REGISTER_MODEL_PLUGIN(SeawolfControl)
}