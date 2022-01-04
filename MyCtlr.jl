module MyCtlr 

using RigidBodyDynamics

Kp = 10.
Kd = 1.

# ------------------------------------------------------------------------
#                          SETUP FUNCTIONS
# ------------------------------------------------------------------------
    function ctlr_setup(mechanism, state; time_step=1e-2,)
        set_ctlr_time_step(time_step)
        resettimestep()
        set_mechanism_to_control(mechanism)
        set_first_state(state)
    end

    function set_ctlr_time_step(time_step)
        dt = time_step
    end

    function resettimestep()
        global time_step_ctr = 0
    end

    function set_mechanism_to_control(mechanism)
        global ctlr_mechanism = mechanism
        global base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(ctlr_mechanism)
    end

    function set_first_state(state)
        global ctlr_first_state = state 
        global des_vel = similar(velocity(ctlr_first_state))
        global prev_vels = similar(des_vel)
    end

# ------------------------------------------------------------------------
#                          UTILITY FUNCTIONS
# ------------------------------------------------------------------------
    function impose_torque_limit!(torque, limit)
        if torque > limit
            torque = limit
        elseif torque < -limit
            torque = -limit
        end
    end

# ------------------------------------------------------------------------
#                              CONTROLLER
# ------------------------------------------------------------------------

function simple_control!(torques::AbstractVector, t, state_alpha::MechanismState, hydro_wrenches::Dict{BodyID, Wrench{Float64}})
    # If it's the first time called in the Runge-Kutta, update the control torque
    if rem(time_step_ctr, 4) == 0
        println("New control goes here!!!")

        # Get the desired velocity values for each joint
        get_des_vel!(des_vel, t)

        tau_base = MyCtlr.base_P_ctlr(torques[velocity_range(state_alpha, base_joint)][1], t, velocity(state_alpha, base_joint), base_joint, des_vel)
    
        torques[velocity_range(state_alpha, base_joint)] .= tau_base
        torques[velocity_range(state_alpha, shoulder_joint)] .= 0 #tau[shoulder_joint][1]
        torques[velocity_range(state_alpha, elbow_joint)] .= 0.0 #tau[elbow_joint][1]
        torques[velocity_range(state_alpha, wrist_joint)] .= 0.0 #tau[wrist_joint][1]
    end

    # push!(ctlr_taus, tau_base)
    # push!(times, t)

    global time_step_ctr += 1
    println("control torques: ", torques)
end;

function get_des_vel!(des_vel, t)
    # TODO: implement a simple pt a to pt b trajectory generation
    des_vel[base_joint][1] = 1.0
    des_vel[shoulder_joint][1] = 0.0
    des_vel[elbow_joint][1] = 0.0 #vdot[elbow_joint][1]
    des_vel[wrist_joint][1] = 0.00
end;
    
function base_P_ctlr(torque, t, vel_act, joint_name, des_vel)
    # println("current time step: ", time_step_ctr)
    # println("remainder after div 4: ", rem(time_step_ctr, 4))
    # println(des_vel[joint_name][1])
    # println("vel_act: ", vel_act)
    vel_error = vel_act[1] - des_vel[joint_name][1]
    d_tau = -Kp*vel_error

    println("At t = ", t, ", actual vel = ", vel_act)

    # Can only change torque a small amount per time step
    if d_tau < -.1
        d_tau = -0.1
    elseif d_tau > 0.1
        d_tau = 0.1
    end
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, 9.0)
    
    return new_tau
end


end