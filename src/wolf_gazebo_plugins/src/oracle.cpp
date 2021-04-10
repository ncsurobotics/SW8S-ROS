#include <functional>
#include <cmath>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{

    struct PositionGoal
    {
        physics::ModelPtr model;
        bool flag;
    };

    class Oracle : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->world = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Oracle::onUpdate, this));
            this->robot = world->ModelByName("seawolf8");

            for (physics::ModelPtr m : this->world->Models())
            {
                if (m.get()->GetName().find("position_goal") != std::string::npos)
                {
                    struct PositionGoal p;
                    p.model = m;
                    p.flag = false;
                    positionGoals.push_back(p);
                }
            }
        }

    public:
        void onUpdate()
        {
            for (struct PositionGoal p : positionGoals)
            {
                if (!p.flag)
                {
                    if (p.model->CollisionBoundingBox().Intersects(robot->CollisionBoundingBox()))
                    {
                        p.flag = true;
                        std::cout << "Position Goal " << p.model->GetName() << " has been triggered" << std::endl;
                    }
                }
            }
        }

    private:
        physics::WorldPtr world;
        physics::ModelPtr robot;
        std::vector<struct PositionGoal> positionGoals;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_WORLD_PLUGIN(Oracle)
}