% averageValue = mean(all_ind(all_ind ~= 0));
% 
% % Replace zeros with the average value
% all_ind(all_ind == 0) = averageValue;

close all
% Combine them into a single matrix for boxplotting
data = [all_ind1(:), all_ind2(:), all_ind3(:), all_ind5(:)];

% Create the boxplot with adjusted whisker length
boxplot(data, 'Labels', {'1%', '2%', '3%', '5%'}, 'Whisker', 5);

% Adding labels with specified font size
xlabel('Percent of Variation', 'FontSize', 18);
ylabel('Number of Iterations', 'FontSize', 18);

% Optional: Adding a title with specified font size


% Set the font size for the tick labels on both axes
set(gca, 'FontSize', 18);