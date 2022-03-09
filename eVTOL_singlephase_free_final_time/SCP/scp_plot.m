figure
plot(Conv_I,Conv_x, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta x (m)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_z, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta z (m)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_vx, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta v_x (m/s)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_vz, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta v_z (m/s)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Obj*1e9, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('Objective value','FontSize',18)
set(gca,'FontSize',16)
grid on

% %------------------------------- States -----------------------------------
x_tf     = State_all(1*N,1:Index+1)'
z_tf     = State_all(2*N,1:Index+1)'
vx_tf    = State_all(3*N,1:Index+1)'
vz_tf    = State_all(4*N,1:Index+1)'

x     = value(X(1,:))';
z     = value(X(2,:))';
vx    = value(X(3,:))';
vz    = value(X(4,:))';
u1    = value(U(1,:))';
u2    = value(U(2,:))';
u3    = value(U(3,:))';
% 
% % z vs. x
figure
plot(x,z,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Altitude (m)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% vx vs. x
figure
plot(x,vx,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Along-Track Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% vz vs. x
figure
plot(x,vz,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Vertical Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% v vs. x
figure
plot(x,sqrt(vx.^2+vz.^2),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% T
figure
plot(x(1:N-1),u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Thrust (N)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% theta
figure
plot(x(1:N-1),asin(u1(1:N-1)./u3(1:N-1))*180/pi,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Theta (deg)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% u1^2+u2^2-u3^2
figure
plot(x(1:N-1),sqrt(u1(1:N-1).^2+u2(1:N-1).^2)-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('u1^2+u2^2-u3^2', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

%% cold to warm
Colors = zeros(Index,3);
Colors(:,1) = linspace(0,1,Index);
Colors(:,3) = linspace(1,0,Index);
half = round(Index/2);
Colors(1:half,2)=linspace(0,1,half);
Colors(half+1:Index,2)=linspace(1,0,Index-half);

x_all     = zeros(N,Index+1);
z_all     = zeros(N,Index+1);
vx_all    = zeros(N,Index+1);
vz_all    = zeros(N,Index+1);
u1_all    = zeros(N,Index);
u2_all    = zeros(N,Index);
u3_all    = zeros(N,Index);
for i = 1:Index+1
    x_all(:,i)     = State_all(1:N,i);
    z_all(:,i)     = State_all(N+1:2*N,i);
    vx_all(:,i)    = State_all(2*N+1:3*N,i);
    vz_all(:,i)    = State_all(3*N+1:4*N,i);
end
for i = 1:Index
    u1_all(:,i)    = Control_all(1:N,i);
    u2_all(:,i)    = Control_all(N+1:2*N,i);
    u3_all(:,i)    = Control_all(2*N+1:3*N,i);
end

% x ~ t
figure
for i = 1:Index
    plot(tau/60, x_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, x_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track distance (m)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% z ~ t
figure
for i = 1:Index
    plot(tau/60, z_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, z_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Altitude (m)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% vx ~ t
figure
for i = 1:Index
    plot(tau/60, vx_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, vx_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track airspeed (m/s)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% vx ~ t
figure
for i = 1:Index
    plot(tau/60, vz_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, vz_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Vertical airspeed (m/s)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u1 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u1_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u1_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track control component (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u2 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u2_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u2_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Vertical control component (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u3 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u3_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u3_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Net thrust (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% theta ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, asin(u1_all(1:N-1,i)./u2_all(1:N-1,i))*180/pi, 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, asin(u1_all(1:N-1,i)./u2_all(1:N-1,i))*180/pi, 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Pitch angle (deg)','FontSize',18)
set(gca,'Fontsize',16)
grid on

%% CPU time
figure
bar(CPU_time)
xlabel('Iteration number','FontSize',18)
ylabel('CPU time (s)','FontSize',18)
set(gca,'Fontsize',16)
grid on
