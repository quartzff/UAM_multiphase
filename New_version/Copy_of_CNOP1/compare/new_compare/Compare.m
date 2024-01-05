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
plot(tG, xG, 'k-x', 'markersize', 10, 'linewidth', 1.5);
hold on
plot(tS, xS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 24);
ylabel('Along-track distance (m)', 'FontSize', 24);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',24);
set(gca,'FontSize',24);
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
plot(tG, zG, 'k-x', 'markersize', 15, 'linewidth', 1.5);
hold on
plot(tS, zS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 18);
ylabel('Altitude (m)', 'FontSize', 18);
xlim([0 25]);
ylim([0 600]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',18);
set(gca,'FontSize',18);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_altitude','-dpdf','-r0')



% vx vs. t
h = figure;
plot(tG, vxG, 'k-x', 'markersize', 15, 'linewidth', 1.5);
hold on
plot(tS, vxS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Time (min)', 'FontSize', 18);
ylabel('Along-track airspeed (m/s)', 'FontSize', 24);
xlim([0 25]);
leg = legend('GPOPS', 'SCP');
legend('Location','southwest')
set(leg,'FontSize',18);
set(gca,'FontSize',18);
grid on
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'conop1_alongspeed','-dpdf','-r0')

figure
plot(tG(90 :156), vxG(90 :156), 'k-x', 'markersize', 15, 'linewidth', 1.5);
hold on
plot(tS(41:100), vxS((41:100)), 'r-o', 'markersize', 7, 'linewidth', 1.5);
set(gca,'FontSize',18);
xlim([23 25]);
grid on
% figure
% plot(tG(105 :156), vxG(105 :156), 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS(99:150), vxS((99:150)), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Along-track airspeed (m/s)', 'FontSize', 18);
% leg = legend('GPOPS', 'SCP');
% set(leg,'FontSize',16);
% set(gca,'FontSize',16);
% grid on


% % vz vs. t
% h = figure;
% plot(tG, vzG, 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS, vzS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Vertical airspeed (m/s)', 'FontSize', 18);
% xlim([0 25]);
% leg = legend('GPOPS', 'SCP');
% legend('Location','southwest')
% set(leg,'FontSize',18);
% set(gca,'FontSize',18);
% grid on
% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'conop1_verticalspeed','-dpdf','-r0')
% 



figure
plot(tG(90 :156), vzG(90 :156), 'k-x', 'markersize', 15, 'linewidth', 1.5);
hold on
plot(tS(46:100), vzS((46:100)), 'r-o', 'markersize', 7, 'linewidth', 1.5);
set(gca,'FontSize',26);
current_xlim = xlim;
% Set the new x-axis limits, keeping the current start point
xlim([current_xlim(1), 25]);
% Make the plot border bold
set(gca, 'linewidth', 2);
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

% % T vs. t
% h = figure;
% plot(tG, TG, 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS, TS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Net thrust (N)', 'FontSize', 18);
% xlim([0 25]);
% ylim([0 5000])
% leg = legend('GPOPS', 'SCP');
% legend('Location','southwest')
% set(leg,'FontSize',18);
% set(gca,'FontSize',18);
% grid on
% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'conop1_thrust','-dpdf','-r0')
% 
% % T vs. t
% figure
% plot(tG(90 :156), TG(90 :156), 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS(41 :100), TS((41 :100)), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlim([23 25])
% ylim([0 5000])
% 
% set(gca,'FontSize',18);
% grid on

% % Theta vs. t
% h = figure;
% plot(tG, thetaG, 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS, thetaS, 'r-o', 'markersize', 7, 'linewidth', 1.5);
% xlabel('Time (min)', 'FontSize', 18);
% ylabel('Pitch angle (deg)', 'FontSize', 18);
% xlim([0 25]);
% leg = legend('GPOPS', 'SCP');
% legend('Location','southwest')
% set(leg,'FontSize',18);
% set(gca,'FontSize',18);
% %legend('on');
% grid on
% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'conop1_pitch','-dpdf','-r0')
% 
% % Zoom-in Theta vs. t
% figure
% plot(tG(90 :156), thetaG(90 :156), 'k-x', 'markersize', 15, 'linewidth', 1.5);
% hold on
% plot(tS(41 :100), thetaS(41:100), 'r-o', 'markersize', 7, 'linewidth', 1.5);
% % Removed x and y labels
% % Increased fontsize for axis ticks
% set(gca,'FontSize',18);
% xlim([23 25])
% % Removed the legend
% legend('off');
% grid on

