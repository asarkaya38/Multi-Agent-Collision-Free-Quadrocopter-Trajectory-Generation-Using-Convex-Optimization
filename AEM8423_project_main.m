%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AEM 8423 Convex Optimization %
%         Project              %
% Ahmet Semi ASARKAYA 5561648  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;
%%%%%%%%%% Start of building QP constraints %%%%%%%%%%
tic
g = [0 ;0 ; -9.81] ; % gravitational acceleration [m/s^2]
%% Define the Size of the Problem
N  = 8;           % # of agents
tf = 17;          % simulation time in seconds
dt = 0.2;         % size of the time steps
T  = tf / dt + 1; % Total time steps
t = 0 : dt : tf;
R = 0.5;          % Allowed safety distance
%% Acceleration and Jerk Limits
ax_max = 10.913;%100.0313; %[m/s^2] % 
ay_max = 10.913;%100.0313;          % .913;%
az_max = 10.913;%100.0313;          % .913;%
jx_max = 2.29;%100.09;   %[m/s^3] %.29;%
jy_max = 2.29;%100.09;            %.29;%
jz_max = 2.29;%100.09;            %.29;%

%% Size of the Lab Environment
l = 6;%16 * .3285; % length in meters
w = 6;%12 * .3285; % width in meters
h = 4;%4;          % height in meters

R = 1;         % Minimum distance to keep between the agents
spacing = R * 1.2;  % Initial spacing between the agents in meters
safe_dist = 0.8 ; % percantage of the lab environment to be used (for safety)

nc = floor((safe_dist * w) / spacing);    % max # of agents in a column (Width)   
nr = floor((safe_dist * l) / spacing / 2) ;% max # of agents in a row (Length)
n_cap = nr * nc ;                         % agent capacity of the room

%% Define Initial and Final States (state_boundaries)
h_fin = 3;
random_final = 0; %Flag for determining if the final positions should be random
[p_init, p_fin, v_init, v_fin, a_fin] = init_final_states(random_final, l, w, N, nc, n_cap, spacing, safe_dist, h_fin);

%% Initial and Final Equality Constraints (boundary_constraints)
[Aeq, beq, H, f] = init_final_boundary_constraints(N, T, dt, tf, p_init, p_fin, v_init, v_fin, a_fin, g);

%% Construct Inequality Constraints (construct_ineq_matrices)
[lb, ub, A, b] = build_ineq_matrices(dt, T, N, ax_max, ay_max, az_max, jx_max, jy_max, jz_max);

%% Quadratic Programming                               
[a_all, cost, exitflag, output] = quadprog(H, f, A, b, Aeq, beq, lb, ub); % Output is the acceleration of each agent at each time step
                                                       % in the order s.t acceleration of 1. agent at each time, then 2. and so on
%% Generate the Trajectories(generate_trajectories)
[pos_all, vel_all,  A_pos, ~] = trajectories(a_all, dt, tf, N, T, p_init, v_init); 
toc
plot_gen(t, tf, N, T, l, w, h, pos_all, vel_all, p_init, v_init, p_fin, v_fin);
tic
%% Sequantial Quadratic Programming with Collision Avoidance
[A_new, b_new] = gen_col_avoid_constraints(A, b, A_pos, T, tf, N, R, pos_all, vel_all, p_init, v_init);
[a_all_new, cost_new, exitflag_new, output_new] = quadprog(H, f, A_new, b_new, Aeq, beq, lb, ub);
[pos_all, vel_all,  A_pos,  A_vel] = trajectories(a_all_new, dt, tf, N, T, p_init, v_init);

cost_prev = 0;
count = 0;
while abs(cost_new - cost_prev) > 0.1
    count = count + 1;
    cost_prev = cost_new;
    [A_new, b_new] = gen_col_avoid_constraints(A, b, A_pos, T, tf, N, R, pos_all, vel_all, p_init, v_init);
    [a_all_new, cost_new, exitflag_new, output_new] = quadprog(H, f, A_new, b_new, Aeq, beq, lb, ub);
    [pos_all, vel_all,  A_pos,  A_vel] = trajectories(a_all_new, dt, tf, N, T, p_init, v_init);
end
toc
plot_gen(t, tf, N, T, l, w, h, pos_all, vel_all, p_init, v_init, p_fin, v_fin);


