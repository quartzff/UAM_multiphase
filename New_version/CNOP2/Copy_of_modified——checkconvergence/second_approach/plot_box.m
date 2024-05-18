
figure
integral_values = integral_values(:);
all_obj = all_obj(:);

% Combine the data into a single column vector
data = [integral_values; all_obj];

% Create a grouping variable
group = [repmat({'GPOPS'}, length(integral_values), 1); repmat({'SCP'}, length(all_obj), 1)];

% Draw the boxplot
boxplot(data, group, 'Labels', {'GPOPS', 'SCP'});
ylabel('Objective Value');