% To automate the initial and final state values,
% I used an initial spacing parameter for initial position, and also, the size of the evironment
% is taken into account. For final position, I basically reflected initial positions
% around a line which goes through on a plane that is right in the middle of the environment.
% I took, initial and, final, velocities/acceleration as zero. Final height is also another
% input to see the results for different heights. This function can easily be customized
% according to desired, initial final state values.
function [p_init, p_fin, v_init, v_fin, a_fin] = init_final_states(random_final, l, w, N, nc, n_cap, spacing, safe_dist, h_fin)
% Initial Position
if N <= nc
    for i = 1 : N
        p_init(i,1) = (1 - safe_dist) / 2 * w + (i - 1) * spacing; % x of the initial positions
    end
    p_init = [p_init ones(N,1)*(1-safe_dist)/2*l zeros(N,1)]; % all location info
elseif N <= n_cap
    for i = 1 : N
        p_init(i,1) = (1 - safe_dist) / 2 * w + ceil(mod(i-0.1,nc) - 1) * spacing; % x of the initial positions
        p_init(i,2) = (1 - safe_dist) / 2 * l + floor((i-0.1)/nc) * spacing; % y of the initial positions
    end
    p_init = [p_init zeros(N,1)]; % all location info
else
    disp("More number of agents than the capacity of the lab environment")
end

%Final Position
if random_final
    p_fin = rand(N,3) * 5;
else
    p_fin = [p_init(:,1) w-p_init(:,2) ones(N,1)*h_fin];
end
 

% Initial/Final Velocity & Acceleration
v_init = zeros(N,3); % Initial velocities of the agents
v_fin = zeros(N,3);  % Final velocities of the agents
a_fin = zeros(N,3);  % Final accelerations of the agents
end