data_scp_all.mat

figure;
hold on;
for i = 1:numTrails
    plot(all_x(:,i), all_z(:,i), 'Color', colors(i,:));
end
xlabel('Along track distance (m)','FontSize',18)
ylabel('Altitude (m)','FontSize',18)
set(gca,'Fontsize',19)
hold off;