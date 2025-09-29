#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class BoxMovementPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Store the initial pose
        this->initialPose = this->model->WorldPose();

        // Get parameters from SDF
        if (_sdf->HasElement("direction"))
        {
            std::string direction = _sdf->Get<std::string>("direction");
            if (direction == "x")
                this->direction = ignition::math::Vector3d(1, 0, 0);
            else if (direction == "y")
                this->direction = ignition::math::Vector3d(0, 1, 0);
            else if (direction == "z")
                this->direction = ignition::math::Vector3d(0, 0, 1);
        }
        else
        {
            this->direction = ignition::math::Vector3d(1, 0, 0); // Default to X direction
        }

        if (_sdf->HasElement("distance"))
            this->distance = _sdf->Get<double>("distance");
        else
            this->distance = 5.0; // Default distance

        if (_sdf->HasElement("speed"))
            this->speed = _sdf->Get<double>("speed");
        else
            this->speed = 1.0; // Default speed

        if (_sdf->HasElement("delay"))
            this->delay = _sdf->Get<double>("delay");
        else
            this->delay = 0.0; // Default delay

        // Listen to the update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&BoxMovementPlugin::OnUpdate, this, _1));

        // Initialize movement state
        this->currentDistance = 0.0;
        this->movingForward = true;
        this->startTime = this->model->GetWorld()->SimTime().Double() + this->delay;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        double currentTime = this->model->GetWorld()->SimTime().Double();
        
        // Check if delay has passed
        if (currentTime < this->startTime)
            return;

        // Calculate movement
        double dt = 0.01; // Assume 100Hz update rate
        double movement = this->speed * dt;

        if (this->movingForward)
        {
            this->currentDistance += movement;
            if (this->currentDistance >= this->distance)
            {
                this->currentDistance = this->distance;
                this->movingForward = false;
            }
        }
        else
        {
            this->currentDistance -= movement;
            if (this->currentDistance <= 0.0)
            {
                this->currentDistance = 0.0;
                this->movingForward = true;
            }
        }

        // Apply movement
        ignition::math::Vector3d offset = this->direction * this->currentDistance;
        ignition::math::Pose3d newPose = this->initialPose;
        newPose.Pos() += offset;
        
        this->model->SetWorldPose(newPose);
    }

private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Movement parameters
    ignition::math::Vector3d direction;
    double distance;
    double speed;
    double delay;

    // Movement state
    ignition::math::Pose3d initialPose;
    double currentDistance;
    bool movingForward;
    double startTime;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BoxMovementPlugin)
}
