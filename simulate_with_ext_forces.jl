using RigidBodyDynamics.OdeIntegrators
import RigidBodyDynamics: default_constraint_stabilization_gains
import RigidBodyDynamics
import RigidBodyDynamics.cache_eltype

function simulate_with_ext_forces(state0::MechanismState{X}, final_time, control! = zero_torque!;
        Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
    T = cache_eltype(state0)
    result = DynamicsResult{T}(state0.mechanism)
    control_torques = similar(velocity(state0))
    closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            control!(control_torques, t, state)
            dynamics!(result, state, control_torques; stabilization_gains=stabilization_gains)
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