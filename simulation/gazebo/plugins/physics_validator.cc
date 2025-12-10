#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class PhysicsValidator : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the world
      this->world = _parent;

      // Check gravity value
      math::Vector3 gravity = this->world->GetPhysicsEngine()->GetGravity();
      printf("PhysicsValidator: Gravity set to [%.3f, %.3f, %.3f]\n", 
             gravity.x, gravity.y, gravity.z);

      // Verify gravity is approximately 9.81 m/s^2
      double expectedGravity = -9.81;
      double tolerance = 0.01; // 1% tolerance
      if (abs(gravity.z - expectedGravity) < tolerance)
      {
        printf("PhysicsValidator: Gravity validation PASSED - within %.3f m/s^2 tolerance\n", tolerance);
      }
      else
      {
        printf("PhysicsValidator: Gravity validation FAILED - expected %.3f, got %.3f\n", expectedGravity, gravity.z);
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PhysicsValidator::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Do nothing for now, but this is where physics validation could occur
      // in real-time during simulation
    }

    // Pointer to the world
    private: physics::WorldPtr world;

    // Connection to the world update event
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(PhysicsValidator)
}