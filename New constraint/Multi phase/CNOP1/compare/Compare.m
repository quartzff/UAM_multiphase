% Compare results from the folders "SCP" and "Verify_gpops_convex" with
% nonconvex control constraint u2^2+u3^2=u4

clear
clc
close all
format short

% parameters

load data_gpops.mat
load data_scp.mat

tG = tG/60;
tS = tS/60;
thetaS = atan(u1S./u2S)*180/pi;
% x vs. t
figure
plot(tG, xG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, xS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Along-track distance (m)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',20);
set(gca,'FontSize',20);
set(gcf,'PaperUnits','inches');
set(gcf,'PaperSize',[8.5 7.5]);
grid on

%z vs. t
% figure
% plot(tG, zG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
% hold on
% plot(tS, zS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 22);
% ylabel('Altitude (m)', 'FontSize', 22);
% xlim([0 25]);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',22);
% set(gca,'FontSize',22);
% grid on
%z vs. t
h = figure;
plot(tG, zG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, zS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Altitude (m)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',22);
set(gca,'FontSize',22);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_altitude','-dpdf','-r0')



% vx vs. t
h = figure;
plot(tG, vxG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, vxS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Along-track airspeed (m/s)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',20);
set(gca,'FontSize',20);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_alongspeed','-dpdf','-r0')

% vz vs. t
h = figure;
plot(tG, vzG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, vzS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Vertical airspeed (m/s)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',20);
set(gca,'FontSize',20);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_verticalspeed','-dpdf','-r0')
% % % u1 vs. t
% % figure
% % 
% % plot(tG, u1G, 'k-*', 'markersize', 7, 'linewidth', 1.5);
% % hold on
% % plot(tS, u1S, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% % xlabel('Time (min)', 'FontSize', 18);
% % ylabel('u_1 (N)', 'FontSize', 18);
% % leg = legend('GPOPS', 'SCP');
% % set(leg,'FontSize',16);
% % set(gca,'FontSize',16);
% % grid on

% % % u2 vs. t
% % figure
% % plot(tG, u2G, 'k-*', 'markersize', 7, 'linewidth', 1.5);
% % hold on
% % plot(tS, u2S, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% % xlabel('Time (min)', 'FontSize', 18);
% % ylabel('u_2 (N)', 'FontSize', 18);
% % leg = legend('GPOPS', 'SCP');
% % set(leg,'FontSize',16);
% % set(gca,'FontSize',16);
% % grid on

% % % u3 vs. t
% % figure
% % plot(tG, u3G, 'k-*', 'markersize', 7, 'linewidth', 1.5);
% % hold on
% % plot(tS, u3S, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% % xlabel('Time (min)', 'FontSize', 18);
% % ylabel('u_3 (N)', 'FontSize', 18);
% % leg = legend('GPOPS', 'SCP');
% % set(leg,'FontSize',16);
% % set(gca,'FontSize',16);
% % grid on

% T vs. t
h = figure;
plot(tG, TG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, sqrt(TS), 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Net thrust (N)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',20);
set(gca,'FontSize',20);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_thrust','-dpdf','-r0')

% % T vs. t
% figure
% plot(tG(250:326), TG(250:326), 'k-*', 'markersize', 7, 'linewidth', 1.5);
% hold on
% plot(tS(72:150), sqrt(TS((72:150))), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Net thrust (N)', 'FontSize', 18);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',16);
% set(gca,'FontSize',16);
% grid on

% Theta vs. t
h = figure;
plot(tG, thetaG, 'k-*', 'markersize', 7, 'linewidth', 1.5);
hold on
plot(tS, thetaS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 20);
ylabel('Pitch angle (deg)', 'FontSize', 20);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',20);
set(gca,'FontSize',20);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_pitch','-dpdf','-r0')
% 
% % Theta vs. t
% figure
% plot(tG(250:326), thetaG(250:326), 'k-*', 'markersize', 7, 'linewidth', 1.5);
% hold on
% plot(tS(72 :150), thetaS(72:150), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Pitch angle (deg)', 'FontSize', 18);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',16);
% set(gca,'FontSize',16);
% grid on

