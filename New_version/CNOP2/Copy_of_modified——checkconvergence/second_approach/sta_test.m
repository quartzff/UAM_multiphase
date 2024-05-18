% Assuming all_obj is your 1x300 array
first_half = all_obj(1:150);
second_half = all_obj(151:end);

% Visual comparison using box plots
figure; % Create a new figure
boxplot([first_half(:).*10^9, second_half(:).*10^9], 'Labels', {'First 150', 'Second 150'});
title('Distribution Comparison');
ylabel('Objective Values');
xlabel('Data Segments');
set(gca, 'FontSize', 18); % Set font size to 18

% Statistical comparison using Mann-Whitney U test
[p, h, stats] = ranksum(first_half, second_half);

% Display the results
fprintf('Mann-Whitney U test p-value: %f\n', p);
if h == 0
    fprintf('No significant difference between the two distributions (at 5%% significance level).\n');
else
    fprintf('Significant difference found between the two distributions (at 5%% significance level).\n');
end
