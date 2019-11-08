#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"


DEFINE_double(target_realtime_rate, 0,
        "Desired rate relative to real time (usually between 0 and 1)."
        "This is documented in Simulator::set_target_realtime_rate().");
DEFINE_double(simulation_time, 0.2 * 25, "Simulation duration in seconds");
DEFINE_double(
        time_step, 1.0E-3,
        "This fixed-tiem step period (in seconds) of discrete updates for the"
        "multibody plant modeled as a discrete system. Strictly positive.");
DEFINE_double(penetration_allowance, 1.0E-3, "Allowance penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-3,
        "Allowance drift speed duration stiction (m/s).");


namespace drake{
namespace examples{
namespace musashiwhole{
namespace {


using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;

int do_main(){
    if (FLAGS_time_step <= 0){
        throw std::runtime_error(
            "time_step must be a strictly postive number. Only the time-stepping"
            "model is supported for this model.");
    }

    // Build a generic multibody plant;
    systems::DiagramBuilder<double> builder;
    auto pair = AddMultibodyPlantSceneGraph(
            &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
    MultibodyPlant<double>& plant = pair.plant;


    const std::string full_name = 
        FindResourceOrThrow("drake/examples/musashiwhole/musashiwhole_model/MUSASHIWHOLE.urdf");
    multibody::Parser(&plant).AddModelFromFile(full_name);

    // Add model of the ground.
    const double static_friction = 1.0;
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(), geometry::HalfSpace(), "GroundVisualGeometry", green);

    // For a time-stepping model only static friction is used.
    const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(), geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);

    plant.Finalize();
    plant.set_penetration_allowance(FLAGS_penetration_allowance);


    // Set the speed tolerance (m/s) for the underlying Stribeck friction model
    // For two points in contact, this is the maximum allowable drift speed at the
    // edge of the friction cone, an approximation to true stiction.
    plant.set_stiction_tolerance(FLAGS_stiction_tolerance);


    // Sanity check model size
    // DRAKE_DEMAND(plant.num_velocities() == );
    // DRAKE_DEMAND(plant.num_position() == )

    // Verify the "PELVIS_LINK" body is free and modeled with quaternions dofs before
    // moving on with that assumption.
    const drake::multibody::Body<double>& pelvis = plant.GetBodyByName("PELVIS_LINK");
    DRAKE_DEMAND(pelvis.is_floating());
    DRAKE_DEMAND(pelvis.has_quaternion_dofs());
    // Since there is a single floating body, we know that the position for it
    // lie first in the state vector.
    DRAKE_DEMAND(pelvis.floating_positions_start() == 0);
    // Similarly for velocities. The velocities for this floating pelvis are the
    // first set of velocities after all model position, since the state vector
    // is stacked as x = [q; v].
    DRAKE_DEMAND(pelvis.floating_velocities_start() == plant.num_positions());

    // Publish contact results for visualization.
    ConnectContactResultsToDrakeVisualizer(&builder, plant);


    geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    systems::Context<double>& plant_context = 
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());


    const VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
    plant.get_actuation_input_port().FixValue(&plant_context, tau);

    // Set the pelvis frame P initial pose.
    const Translation3d X_WP(0.0, 0.0, 5.0);
    plant.SetFreeBodyPoseInWorldFrame(&plant_context, pelvis, X_WP);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(FLAGS_simulation_time);


    return 0;
}

}   // namespace
}   // namespace musashiwhole
}   // namespace examples
}   // namespace drake


int main(int argc, char* argv[]){
    gflags::SetUsageMessage(
            "\nPassive simulation of the musashiwhle robot. With the defalut time step of"
            "\n1 ms, this simulation typically runs slightly faster than real time."
            "\nThe time step has an effect on the joint limits stiffness, which"
            "\nconverge quadratically to the rigid limit as the time step is"
            "\ndecresed. Thus, decreased the time step for more accurately resolved"
            "\njoint limits."
            "\nLaunch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return drake::examples::musashiwhole::do_main();

}
