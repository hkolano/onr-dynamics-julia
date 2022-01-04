module MyCtlr 

using RigidBodyDynamics
time_step_ctr = 0

Kp = 10.
Kd = 1.

function set_ctlr_time_step(time_step)
    dt = time_step
end
    
function joint_level_P_ctlr_v2(torque, t, vel_act, joint_name, des_vel)

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
    if new_tau > 9.0
        new_tau = 9.0
    elseif new_tau < -9.0
        new_tau = -9.0
    end

    # time_step_ctr = time_step_ctr + 1
    
    # println("OOGABOOGA!!")
    return new_tau
end


end