figure



% boxplot(all_obj.*10^9,'Whisker', 3)
% set(gca, 'XTickLabel', {});
% xlabel('SCP ±10% variation', 'FontSize', 20);
% ylabel('Objective value', 'FontSize', 20);
% set(gca,'FontSize',20);


% boxplot(integral_values,'Whisker', 5)
% set(gca, 'XTickLabel', {});
% xlabel('GPOPS ±5% variation', 'FontSize', 20);
% ylabel('Objective value', 'FontSize', 20);
% set(gca,'FontSize',20);



figure

boxplot(CPU_time,'Whisker', 2)
set(gca, 'XTickLabel', {});
xlabel('GPOPS ±5% variation', 'FontSize', 20);
ylabel('Computational time(s)', 'FontSize', 20);
set(gca,'FontSize',20);
% 
% 
