function [lb, ub, A, b] = build_ineq_matrices(dt, T, N, ax_max, ay_max, az_max, jx_max, jy_max, jz_max)
% Acceleration Inequality Constraints
lb = [];
ub = [];
for i = 1 : T*N
    lb = [lb ; -ax_max ;-ay_max ;-az_max];
    ub = [ub ; ax_max ;ay_max ;az_max];
end

% Jerk Inequality Constraints
A_max_n = []; % Jerk inequality matrix for each agent
A_max   = []; % Global jerk inequality
b_max_n = [];
b_max   = [];
for i = 1 : T-1
    A_max_nh = [];
    for j = 1 : T-1
        if i == j
            A_max_nh = [A_max_nh  -eye(3)];
        elseif i == j + 1
            A_max_nh = [A_max_nh  -eye(3)];
        else
            A_max_nh = [A_max_nh  zeros(3,3)];
        end
    end
    A_max_n = [A_max_n ; A_max_nh / dt];
    b_max_n = [b_max_n ; jx_max; jy_max; jz_max];
end
A_max_n = [A_max_n zeros((T-1)*3,3)];
for i = 1 : N
    A_maxh = [];
    for j = 1 : N
        if i == j
            A_maxh = [A_maxh A_max_n];
        else
            A_maxh = [A_maxh zeros(size(A_max_n))];
        end
    end
    A_max = [A_max ;A_maxh];
    b_max = [b_max ;b_max_n];
end

A = [A_max; -A_max];
b = [b_max; b_max];
end