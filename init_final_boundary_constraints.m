function [Aeq, beq, H, f] = init_final_boundary_constraints(N, T, dt, tf, p_init, p_fin, v_init, v_fin, a_fin, g)
g_all = g;
for i = 1 : N*T - 1
    g_all = [g_all ;g];
end

Aeq   = [];
% Aeq_1 = [eye(3) zeros(3, 3*T-3)];
Aeq_2 = []; % For final positions
Aeq_3 = []; % For final velocities
for i = 1 : T - 1
    Aeq_2 = [Aeq_2 eye(3) * dt^2 * (2 * T - (2 * i + 1)) / 2];
    Aeq_3 =[Aeq_3 eye(3) * dt];
end

Aeq_2 = [Aeq_2 eye(3) * 0];
Aeq_3 = [Aeq_3 eye(3) * 0];
Aeq_4 = [zeros(3, 3*T-3) eye(3)];
Aeq_n = [Aeq_2 ;Aeq_3 ;Aeq_4];

beq = [];
for i = 1 : N
    beq_n = [p_fin(i,:)'-p_init(i,:)'-tf*v_init(i,:)';
        v_fin(i,:)'-v_init(i,:)';
        a_fin(i,:)'];
    beq   = [beq ;beq_n];
    Aeq_r = [];
    for j = 1 : N
        if i == j
            Aeq_r = [Aeq_r Aeq_n];
        else
            Aeq_r = [Aeq_r zeros(size(Aeq_n))];
        end
    end
    Aeq = [Aeq ;Aeq_r];
end

H = eye(size(Aeq,2));
f = -2 * g_all;
end