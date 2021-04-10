#include <functional>
#include <cmath>
#include <vector>
#include <regex>

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
        int order;
        int timer;
    };

    class Oracle : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {

            //find models in the world for use in update loop
            this->world = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Oracle::onUpdate, this));
            this->robot = world->ModelByName("seawolf8");

            this->lastPositionGoalTriggered = 0;

            //extract position goals from a list of all models to split into a struct that is easier to work with
            std::regex rgx("^position_goal_(\\d+)_timer_(\\d+)$");

            for (physics::ModelPtr m : this->world->Models())
            {
                std::smatch match;
                const std::string name = m.get()->GetName();
                if (std::regex_search(name.begin(), name.end(), match, rgx))
                {
                    struct PositionGoal p;
                    p.model = m;
                    p.flag = false;

                    //extract the order and timer for each goal out of the name so the gates can be done in a particular order
                    // and so the robot must hold in that space for a set amount of time
                    p.order = std::stoi(match[1]);
                    p.timer = std::stoi(match[2]);
                    positionGoals.push_back(p);
                }
            }
        }

    public:
        void onUpdate()
        {
            if (!success)
            {
                //are we done checking goals?
                if (lastPositionGoalTriggered < positionGoals.size())
                {
                    struct PositionGoal currentPositionGoal = positionGoals[lastPositionGoalTriggered];

                    if (currentPositionGoal.model->CollisionBoundingBox().Intersects(robot->CollisionBoundingBox()))
                    {
                        currentPositionGoal.flag = true;
                        lastPositionGoalTriggered++;

                        //set color to green
                        transport::NodePtr node;
                        transport::PublisherPtr visPub;
                        msgs::Visual visualMsg = currentPositionGoal.model->GetLink()->GetVisualMessage("visual");
                        msgs::Color* green = new msgs::Color;

                        node = transport::NodePtr(new transport::Node());
                        node->Init(this->world->Name());


                        visualMsg.set_name(currentPositionGoal.model->GetLink()->GetScopedName());
                        visualMsg.set_parent_name(currentPositionGoal.model->GetScopedName());
                        visPub = node->Advertise<msgs::Visual>("/gazebo/default/visual", 10);


                        green->set_r(0.0f);
                        green->set_g(1.0f);
                        green->set_b(0.0f);
                        green->set_a(0.3f);
                        visualMsg.mutable_material()->set_allocated_diffuse(green);
                        visPub->Publish(visualMsg);

                        std::cout << "Position Goal " << currentPositionGoal.order << " has been triggered" << std::endl;
                    }
                }
                else
                {
                    success = true;
                    std::cout << "All Tests Pass!" << std::endl;
                }
            }
        }

    private:
        physics::WorldPtr world;
        physics::ModelPtr robot;
        std::vector<struct PositionGoal> positionGoals;

        //which goal should be checked for right now?
        int lastPositionGoalTriggered;

        //have all targets been met?
        bool success;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_WORLD_PLUGIN(Oracle)
}