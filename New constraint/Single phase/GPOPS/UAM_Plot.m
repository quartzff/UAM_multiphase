%------------------------------%
% Extract Solution from Output %
%------------------------------%
close all

solution = output.result.solution;
t     = solution.phase(1).time;
x     = solution.phase(1).state(:,1);
z     = solution.phase(1).state(:,2);
vx    = solution.phase(1).state(:,3);
vz    = solution.phase(1).state(:,4);
u1    = solution.phase(1).control(:,1);
u2    = solution.phase(1).control(:,2);
u3    = solution.phase(1).control(:,3);

%---------------%
% Plot Solution %
%---------------%
% z vs. x
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
plot(x,sqrt(u3),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Thrust (N)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% theta
figure
plot(x,atan(u1./u2)*180/pi,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Theta (deg)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% u1^2+u2^2-u3^2
figure
plot(x,u1.^2+u2.^2-u3,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('u1^2+u2^2-u3^2', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

%% Save the results to compare with SCP
tG     = t;
xG     = x;
zG     = z;
vxG    = vx;
vzG    = vz;
u1G    = u1;
u2G    = u2;
u3G    = u3;
TG     = u3;
thetaG = asin(u1./u3)*180/pi;
save data_gpops.mat tG xG zG vxG vzG u1G u2G u3G TG thetaG
