using MeshCatMechanisms
using RigidBodyDynamics
using MeshCat
using GeometryBasics

println("Done importing packages.")

vis = Visualizer()
render(vis)

src_dir = dirname(pathof(RigidBodyDynamics))
urdf_file = joinpath(src_dir, "..", "..", "..", "..", "..", "onr-dynamics-julia", "URDFs", "alphaArmFixedJaws.urdf")
mechanism = parse_urdf(urdf_file)

visuals = URDFVisuals(urdf_file)
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf_file), vis)

arrow_vis = ArrowVisualizer(vis)
setobject!(arrow_vis)
settransform!(arrow_vis, Point(0.0, 0.0, 0.0), Vec(0.5, 0.5, 0.0))
