load('data_scp_new.mat')
% Define your time vector 't' and your data vectors 'u1' and 'u2'
t = tS; % Add the rest of your time data here
u1 = u1S; % Add the rest of your u1 data here
u2 = u2S; % Add the rest of your u2 data here

% % Ensure that t, u1, and u2 are all row vectors or all column vectors
% t = t(:);
% u1 = u1(:);
% u2 = u2(:);

% Calculate u1^2 + u2^2 at each time point
u_squared_sum = u1.^2 + u2.^2;

% Use trapz to integrate over time
integral_value = trapz(t, u_squared_sum);

% Display the result
disp('Integral of u1^2 + u2^2 over time: ');
disp(integral_value);