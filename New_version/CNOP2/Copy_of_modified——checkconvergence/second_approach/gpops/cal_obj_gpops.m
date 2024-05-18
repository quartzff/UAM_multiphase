integral_values = zeros(1, 200);%length(all_t));

for k = 1:200%length(all_t)
    t = all_t{k}; % Extract t from the k-th cell of all_t
    T = all_T{k}; % Extract T from the k-th cell of all_T
    
    % Make sure t and T have the same length
    if length(t) ~= length(T)
        error('Length of t and T must be the same in each cell.');
    end
    
    % Calculate the integral of T.^2 with respect to t
    integral_values(k) = trapz(t, T.^2);
end