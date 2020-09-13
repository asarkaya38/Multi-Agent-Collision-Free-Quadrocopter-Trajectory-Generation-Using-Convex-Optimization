function [A_new, b_new] = gen_col_avoid_constraints(A, b, A_pos, T, tf, N, R, pos_all, vel_all, p_init, v_init)
pij_all = [];
vij_all = [];
% Finding pij, vij vectors and dij distance
for k = 1 : T - 1
    for i = 1 : N
        for j = i+1 : N
            pij_all = [pij_all; pos_all((i-1)*T+k+1,:) - pos_all((j-1)*T+k+1,:)]; %pij_all, is in the order of
            vij_all = [vij_all; vel_all((i-1)*T+k+1,:) - vel_all((j-1)*T+k+1,:)]; %ij, and then time 
        end
    end
end
dij_all = vecnorm(pij_all,2,2); % 0.01 for ill condition
pij_all(dij_all <= 0.1,:) = 0.05;
dij_all(dij_all <= 0.1)   = 0.1;

% Construt Collision Avoidance constraints
A_col = zeros((T-1)*N*(N-1)/2,3*T*N);
b_col = zeros((T-1)*N*(N-1)/2,1);
c = 0;
for k = 1 : T-1
    for i = 1 : N
        for j = i+1 : N
            c = c + 1;
           A_col_n_holder = A_pos(((T-1)*3*(i-1)+(k-1)*3+1:(T-1)*3*(i-1)+(k-1)*3+3), :) -...
                            A_pos(((T-1)*3*(j-1)+(k-1)*3+1:(T-1)*3*(j-1)+(k-1)*3+3), :);
           A_col_n = pij_all(c,:) / dij_all(c) * A_col_n_holder;  
           A_col(c,:) = A_col_n; 
           p0_ij      = p_init(i,:) - p_init(j,:);
           v0_ij      = v_init(i,:) - v_init(j,:);
           b_col(c)   = -R + dij_all(c) + pij_all(c,:) / dij_all(c) * (p0_ij' + tf*v0_ij' - pij_all(c,:)');
        end
    end
end
A_col = -A_col;

A_new = [A ; A_col]; % add collision avoidance constraints
b_new = [b ; b_col];
end