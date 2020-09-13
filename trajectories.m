function [pos_all, vel_all,  A_pos,  A_vel] = trajectories(a_all, dt, tf, N, T, p_init, v_init)
A_pos_nn = [];
A_vel_nn = [];
for i = 1 : T-1
    A_pos_n = [];
    A_vel_n = [];
    for j = 1 : i
        A_pos_n = [A_pos_n eye(3) * dt^2 * (2 * (i+1) - (2 * j + 1)) / 2];
        A_vel_n = [A_vel_n eye(3) * dt];    
    end
    for k = i+1 : T-1
         A_pos_n = [A_pos_n zeros(3,3)];
         A_vel_n = [A_vel_n zeros(3,3)];
    end
    A_pos_nn = [A_pos_nn ; A_pos_n];
    A_vel_nn = [A_vel_nn ; A_vel_n];
end

A_pos_nn = [A_pos_nn  zeros((T-1)*3,3)];
A_vel_nn = [A_vel_nn  zeros((T-1)*3,3)];
A_pos = [];
A_vel = [];
for i = 1 : N
    A_pos_psu = [];
    A_vel_psu = [];
    for j = 1 : N
        if i == j
            A_pos_psu = [A_pos_psu A_pos_nn];
            A_vel_psu = [A_vel_psu A_vel_nn];
        else
            A_pos_psu = [A_pos_psu zeros(size(A_pos_nn))];
            A_vel_psu = [A_vel_psu zeros(size(A_vel_nn))];   
        end
    end
    A_pos = [A_pos ; A_pos_psu];
    A_vel = [A_vel ; A_vel_psu];
end

p_init_g = [];
v_init_g = [];
for i = 1 : N
    for j = 1 : T-1
        p_init_g = [p_init_g ; p_init(i,:)'];
        v_init_g = [v_init_g ; v_init(i,:)'];
    end
end

new_pos = p_init_g + tf * v_init_g + A_pos * a_all;
new_vel = v_init_g + A_vel * a_all;

% Formatting for ploting
pos_all = [];
vel_all = [];
for i = 1 : N
    pos_all = [pos_all ; p_init(i,:)];
    vel_all = [vel_all ; v_init(i,:)];
    for j = 1 : T-1
        new_pos_all_psu = [new_pos((i-1)*(T-1)*3+(j-1)*3+1) new_pos((i-1)*(T-1)*3+(j-1)*3+2)...
                               new_pos((i-1)*(T-1)*3+(j-1)*3 +3)];
        pos_all = [pos_all ; new_pos_all_psu];
        new_vel_all_psu = [new_vel((i-1)*(T-1)*3+(j-1)*3+1) new_vel((i-1)*(T-1)*3+(j-1)*3+2)...
                               new_vel((i-1)*(T-1)*3+(j-1)*3 +3)];
        vel_all = [vel_all ; new_vel_all_psu];
    end
end
end