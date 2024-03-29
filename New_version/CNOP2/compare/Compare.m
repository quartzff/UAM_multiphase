% Compare results from the folders "SCP" and "Verify_gpops_convex" with
% nonconvex control constraint u2^2+u3^2=u4

clear
clc
close all
format short

% parameters

load data_gpops.mat
load data_scp_new.mat

tG = tG/60;
tS = tS/60;
thetaS = atan(u1S./u2S)*180/pi;
% x vs. t
figure
plot(tG, xG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, xS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 24);
ylabel('Along-track distance (m)', 'FontSize', 24);
leg = legend('GPOPS', 'SCP');
set(leg,'FontSize',24);
set(gca,'FontSize',24);
grid on

% z vs. t
figure
plot(tG, zG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, zS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 28);
ylabel('Altitude (m)', 'FontSize', 28);
leg = legend('GPOPS', 'SCP');
xlim([0,25])
set(leg,'FontSize',28);
set(gca,'FontSize',28);
grid on

% vx vs. t
figure
plot(tG, vxG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, vxS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 28);
ylabel('Along-track airspeed (m/s)', 'FontSize', 28);
leg = legend('GPOPS', 'SCP');
xlim([0,25])
set(leg,'FontSize',28);
set(gca,'FontSize',28);
grid on

% vz vs. t
figure
plot(tG, vzG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, vzS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 28);
ylabel('Vertical airspeed (m/s)', 'FontSize', 28);
leg = legend('GPOPS', 'SCP');
xlim([0,25])
set(leg,'FontSize',28);
set(gca,'FontSize',28);
grid on

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
figure
plot(tG, TG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, TS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 28);
ylabel('Net thrust (N)', 'FontSize', 28);
leg = legend('GPOPS', 'SCP');
xlim([0,25])
set(leg,'FontSize',28);
set(gca,'FontSize',28);
grid on

% % T vs. t
% figure
% plot(tG(350:473), TG(350:473), 'k-*', 'markersize', 7, 'linewidth', 1.5);
% hold on
% plot(tS(72:116), sqrt(TS((72:116))), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Net thrust (N)', 'FontSize', 18);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',16);
% set(gca,'FontSize',16);
% grid on

% Theta vs. t
figure
plot(tG, thetaG, 'k-*', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, thetaS, 'r-o', 'markersize', 10, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 28);
ylabel('Pitch angle (deg)', 'FontSize', 28);
leg = legend('GPOPS', 'SCP');
xlim([0,25])
set(leg,'FontSize',28);
set(gca,'FontSize',28);
grid on


% % Theta vs. t
% figure
% plot(tG(350:473), thetaG(350:473), 'k-*', 'markersize', 7, 'linewidth', 1.5);
% hold on
% plot(tS(72 :116), thetaS(72:116), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Pitch angle (deg)', 'FontSize', 18);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',16);
% set(gca,'FontSize',16);
% grid on

