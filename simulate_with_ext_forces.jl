using RigidBodyDynamics.OdeIntegrators
import RigidBodyDynamics: default_constraint_stabilization_gains
import RigidBodyDynamics
import RigidBodyDynamics.cache_eltype

function simulate_with_ext_forces(state0::MechanismState{X}, final_time, hydro_calc!, control! = zero_torque!;
        Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
    T = cache_eltype(state0)
    result = DynamicsResult{T}(state0.mechanism)
    control_torques = similar(velocity(state0))
    hydro_wrenches = Dict{BodyID, Wrench{Float64}}()
    closed_loop_dynamics! = let result=result, hydro_wrenches=hydro_wrenches, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            # println("------ NEW SIM STATE -----")
            # println("Current State:")
            # println(configuration(state))
            hydro_calc!(hydro_wrenches, t, state)
            control!(control_torques, t, state, hydro_wrenches, result.v̇)
            # println("Control torques")
            # println(control_torques)

            # hydro_calc!(hydro_wrenches, t, state)
            # println("Hydro wrenches")
            # println(hydro_wrenches)

            dynamics!(result, state, control_torques, hydro_wrenches; stabilization_gains=stabilization_gains)
            # println("result:")
            # println(result.v̇)
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end
tableau = runge_kutta_4(T)
storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
integrate(integrator, final_time, Δt)
storage.ts, storage.qs, storage.vs
end