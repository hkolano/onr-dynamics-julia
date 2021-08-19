using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using MeshCat
using MeshCatMechanisms
using MechanismGeometries

# ------------------------------------------------------------------------
#                            MODEL DEFINITION
# ------------------------------------------------------------------------
src_dir = dirname(pathof(RigidBodyDynamics))
urdf_file = joinpath(src_dir, "..", "..", "..", "..", "..", "onr-dynamics-julia", "alphaArmFixedJaws.urdf")
mechanism = parse_urdf(urdf_file)

visuals = URDFVisuals(urdf_file)

println("URDF parsed. \n")

# ------------------------------------------------------------------------
#                              CONTROLLER
# ------------------------------------------------------------------------

base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(mechanism)

function simple_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, base_joint)] .= 2.0*sin(t)
    torques[velocity_range(state, shoulder_joint)] .= 0.
    torques[velocity_range(state, elbow_joint)] .= 0.
    torques[velocity_range(state, elbow_joint)] .= 0.
end;

# ------------------------------------------------------------------------
#                              SIMULATION
# ------------------------------------------------------------------------
state = MechanismState(mechanism)
zero_velocity!(state)
set_configuration!(state, base_joint, 0.0)
set_configuration!(state, shoulder_joint, 0.0)
set_configuration!(state, elbow_joint, 0.0)
set_configuration!(state, wrist_joint, 0.0)

final_time = 5.
ts, qs, vs = simulate(state, final_time, simple_control!; Δt = 1e-3)

println("Simulation finished.")
# ------------------------------------------------------------------------
#                             VISUALIZATION
# ------------------------------------------------------------------------
vis = Visualizer()
open(vis)

delete!(vis)
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf_file), vis)
# set_configuration!(mvis, [0.0, 0.0])

animation = Animation(mvis, ts, qs)
setanimation!(mvis, animation)

println("\n done.")
