function should_stop = userDefinedStoppingCondition(ee_x, ee_y, ee_vx, ee_vy, t)
    % If the gripper was attached at a displacement of d from the COM, the
    % updated vx(c)​=vx(p)​−ωp​ry​ and vy(c)​=vy(p)​+ωp​rx
    % Furthermore, the angular moementum would (assuming the gripper is fixed)
    % (m_COM/I_COM)*(rxvy(p) - ryvx(p)) this is rotational velocity about COM of object.

    gravity_acc = 9.81;
    target_x_range = 5.0; % This is where I want my final object to land.
    should_stop = false;
    if (ee_vx/gravity_acc)*(ee_vy + sqrt(ee_vx*ee_vx + 2*ee_y*gravity_acc)) == target_x_range - ee_x
        if (ee_vx/target_x_range)>0 % Make sure that manipulator doesnt throw it on the other side
            should_stop = true;
        end
    end
end
