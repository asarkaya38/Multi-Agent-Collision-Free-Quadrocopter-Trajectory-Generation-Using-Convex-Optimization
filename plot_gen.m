function plot_gen(t, tf, N, T, l, w, h, pos_all, vel_all, p_init, v_init, p_fin, v_fin)
movegui(figure('Name','Position vs Time of the Agents','NumberTitle','off'),[145,90]);
tiledlayout(N,3) % For position
for i = 1 : N
    nexttile
    plot(t,pos_all(((i-1)*T+1:T*i),1));
    if i == 1
        title('x [m] vs time');
    end
    y_label = sprintf('%d. Agent',i);
    ylabel(y_label);
    hold on
    scatter(0,p_init(i,1));
    hold on
    scatter(tf,p_fin(i,1),'filled');
    xlim([-0.1, tf+1]);
    ylim([0, w]);
    
    nexttile
    plot(t,pos_all(((i-1)*T+1:T*i),2));
    if i == 1
        title('y [m] vs time');
    end
    if i == N
        xlabel('time [s]');
    end
    hold on
    scatter(0,p_init(i,2));
    hold on
    scatter(tf,p_fin(i,2),'filled');
    xlim([-0.1, tf+1]);
    ylim([0, l]);
    
    nexttile
    plot(t,pos_all(((i-1)*T+1:T*i),3));
    if i == 1
        title('z [m] vs time');
    end
    hold on
    scatter(0,p_init(i,3));
    hold on
    scatter(tf,p_fin(i,3),'filled');
    xlim([-0.1, tf+1]);
    ylim([0, h]);
end

movegui(figure('Name','Velocity vs Time of the Agents','NumberTitle','off'),[720,90]);
tiledlayout(N,3) % For Velocity
for i = 1 : N
    nexttile
    plot(t,vel_all(((i-1)*T+1:T*i),1));
    if i == 1
        title('x [m/s] vs time');
    end
    y_label = sprintf('%d. Agent',i);
    ylabel(y_label);
    hold on
    scatter(0,v_init(i,1));
    hold on
    scatter(tf,v_fin(i,1),'filled');
    xlim([-0.1, tf+1]);
    
    nexttile
    plot(t,vel_all(((i-1)*T+1:T*i),2));
    if i == 1
        title('y [m/s] vs time');
    end
    if i == N
        xlabel('time [s]');
    end
    hold on
    scatter(0,v_init(i,2));
    hold on
    scatter(tf,v_fin(i,2),'filled');
    xlim([-0.1, tf+1]);
    
    nexttile
    plot(t,vel_all(((i-1)*T+1:T*i),3));
    if i == 1
        title('z [m] vs time');
    end
    hold on
    scatter(0,v_init(i,3));
    hold on
    scatter(tf,v_fin(i,3),'filled');
    xlim([-0.1, tf+1]);
end

figure('Name','3D Trajectories','NumberTitle','off');
for i = 1 : N
    %agent_label = sprintf('%d. Agent',i);
    plot3(pos_all(((i-1)*T+1:T*i),1), pos_all(((i-1)*T+1:T*i),2) ,pos_all(((i-1)*T+1:T*i),3));
    hold on
    %legend(agent_label)
end
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
title('Trajectories')
axis equal
xlim([0,w]);
ylim([0,l]);
zlim([0,h]);
grid on
end