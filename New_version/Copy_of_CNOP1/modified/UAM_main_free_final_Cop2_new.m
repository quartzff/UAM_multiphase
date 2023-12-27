%% Minimize control effort for UAM cruise, descent, and landing
% Using trapezoidal discretization, choose X and U as design variables
% Time-varying A; fixed B & b through the whole recursive process

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Preparation                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc
format long

% Parameters
m      = 240;           % vehicle's mass, kg
rho    = 1.225;         % air density, kg/m^3
Sx     = 2.11;          % vehicle's frontal area, m^2
Sz     = 1.47;          % drag coefficient
g      = 9.81;          % gravitaional acceleration, m/s^2
CD     = 1;             % drag coefficient

%---------------------- Initial reference trajectory ----------------------
x0 = 0;         % initial along-track distance, m
z0 = 500;       % initial altitude, m
vx0 = 13.85;    % initial along-track airspeed, m/s
vz0 = 0;        % initial vertical airspeed, m/s

x1 = 20000;         % phase 1 along-track distance, m
z1 = 500;       % phase 1 altitude, m
vx1 = 0;    % phase 1 along-track airspeed, m/s
vz1 = 0;        % phase 1 vertical airspeed, m/s

xf = 20000;     % final along-track distance, m
zf = 0;         % final altitude, m
vxf = 0;        % final along-track airspeed, m/s
vzf = 0;        % final vertical airspeed, m/s

vmax   = 17;    % maximum airspeed
Tmin   = 0;      % minimum net thrust
Tmax   = 4800;   % maximum net thrust, N
thetamin = -6*pi/180;    % minimum rotor tip-path-plane pitch angle
thetamax = 6*pi/180;   % maximum rotor tip-path-plane pitch angle

t0 = 0;
tf = 25*60;     % required time of arrival (RTA), min







%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Modeling & Optimization                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Max_iter = 25;   % Maximum number of iteration
col_points = 50;
col_points2 = 60;
tau = linspace(0,1,col_points)';
tau1 = linspace(0,1,col_points2)';
step = tau(2)-tau(1);
N = length(tau)+length(tau1);   % N nodes
sigma_guess = 24*60; 

sigma2_guess = 3*60;

x1 = linspace(x0,x1,col_points)';
z1 = linspace(z0,z1,col_points)';
vx1 = linspace(vx0,vx1,col_points)';
vz1 = linspace(vz0,vz1,col_points)';

x2 = linspace(x1(end),xf,col_points2)';
z2 = linspace(z1(end),zf,col_points2)';
vx2 = linspace(vx1(end),vxf,col_points2)';
vz2 = linspace(vz1(end),vzf,col_points2)';
 x = [x1;x2];
 z = [z1;z2];
 vx = [vx1;vx2];
 vz = [vz1;vz2];

X = sdpvar(4,N);
U = sdpvar(3,N);
Sigma = sdpvar(1,1);
Sigma2 = sdpvar(1,1);

State_all = zeros(4*N, Max_iter+1);
State_all(:,1) = [x; z; vx; vz];
Control_all = zeros(3*N, Max_iter);
Del_state = zeros(6, Max_iter);
Sigma_all = zeros(1, Max_iter+1);
Sigma2_all = zeros(1, Max_iter+1);
Sigma_all(:,1) = sigma_guess;
Sigma2_all(:,1) = sigma2_guess;
X(:,1) = [x0; z0; vx0; vz0];
% U(:,1) = [0; 0; 0];
sigma = sigma_guess;
sigma2 = sigma2_guess;
u1 = zeros(N,1)*210;
u2 = zeros(N,1)*2300;
u3 = ones(N,1)*2300^2;

%--------------------------- Iteration procedure --------------------------
for Index = 1:Max_iter
    
    Cons = [];
    % Recursively compute each state (A and B are constant for each step)
    J = 0;
    for i = 1:N-1
       % Cons = Cons + [  Sigma == 1500 ];
        % From last step
        x01  = x(i); z01  = z(i); vx01 = vx(i); vz01 = vz(i);
        x02  = x(i+1); z02  = z(i+1); vx02 = vx(i+1); vz02 = vz(i+1);
        u1_01 = u1(i); u2_01 = u2(i); u3_01 = u3(i);
        u1_02 = u1(i+1); u2_02 = u2(i+1); u2_03 = u3(i+1);
        phase_node = col_points;
        if i <=  col_points

        [H1, H2, G1, G2, b1, b2] = ComDyn(x01,z01,vx01, vz01, x02, z02, vx02, vz02, u1_01, u2_01, u1_02, u2_02, sigma, Sigma, step, m, rho, CD, Sx, Sz, g);
        
        %---Linearized dynamics
        Cons = Cons + [ -H1*X(:,i) + H2*X(:,i+1) - G1*U(:,i) - G2*U(:,i+1) == 0.5*step*(b1+b2) ];  % N
%         X(:,i+1) = inv(H2) * (H1*X(:,i) + G1*U(:,i) + G2*U(:,i+1) + 0.5*step*(b1+b2));
        J = J + (1/1e9*step*(sigma*u3_01+sigma*(U(3,i)-u3_01)+u3_01*(Sigma-sigma)));
        
        else 
        [H1, H2, G1, G2, b1, b2] = ComDyn(x01,z01,vx01, vz01, x02, z02, vx02, vz02, u1_01, u2_01, u1_02, u2_02, sigma2, Sigma2, step, m, rho, CD, Sx, Sz, g);
        
        
        %---Linearized dynamics
        Cons = Cons + [ -H1*X(:,i) + H2*X(:,i+1) - G1*U(:,i) - G2*U(:,i+1) == 0.5*step*(b1+b2) ];  % N
        
        J = J + (1/1e9*step*(sigma2*u3_01+sigma2*(U(3,i)-u3_01)+u3_01*(Sigma2-sigma2)));
        end
        %---State constraints
        Cons = Cons + [ x0 <= X(1,i+1) <= xf ];
        Cons = Cons + [ zf <= X(2,i+1) <= z0 ];
        Cons = Cons + [ 0  <= X(3,i+1) <= vmax ];
        Cons = Cons + [ X(1, phase_node) == 20000 ];
        Cons = Cons + [ X(2, phase_node) == z0 ];
        Cons = Cons + [ X(1, phase_node +1) == 20000 ];
        Cons = Cons + [ X(2, phase_node +1) == z0 ];
        Cons = Cons + [ -vmax <= X(4,i+1) <= 0 ];
        Cons = Cons + [ sqrt(X(3,i+1)^2 + X(4,i+1)^2) <= vmax ];
        %Cons = Cons + [ u1_01^2 + u2_01^2 == U(3,i+1) ];
        %Cons = Cons + [ X(3,i+1)^2 + X(4,i+1)^2 <= vmax^2 ];
        
        %---Objective
        %J = J + 1/1e9*step*1/2*U(3,i)^2;
        
    end
    
    %---Objective function(functional)
    Objective = J;
    
    %---Control constraints
    for j = 1:N-1
        Cons = Cons + [ U(2,j)*tan(-thetamax) <= U(1,j) <= U(2,j)*tan(thetamax) ];
        Cons = Cons + [ 0 <= U(3,j) <= Tmax^2 ];
        %Cons = Cons + [ U(3,j) == Tmax^2 ];
        Cons = Cons + [ U(1,j)^2 + U(2,j)^2 <= U(3,j) ];
       % J = J + abs(U(1,j)^2 + U(2,j)^2 - U(3,j));
    end
    Cons = Cons + [  Sigma + Sigma2 == 1500 ];% added time constraint
    Cons = Cons + [ 500 <= Sigma <= 1482 ];% added time constraint
    Cons = Cons + [ 30 <= Sigma2 <= 1000 ];% added time constraint
    %Cons = Cons + [ Sigma == 733.8 ];
    %---Trust-region constraint
    % |X-Xk|<delta
    Xk = [x'; z'; vx'; vz']; % X_k-1
    delta = [2e3*ones(1,N); 5e2*ones(1,N); 50*ones(1,N); 50*ones(1,N)];
    Cons = Cons + [ -delta <= X-Xk <= delta ];
    
    
    delta1 =0.5*sigma_guess ;
    Cons = Cons + [ -delta1 <= Sigma-sigma <= delta1 ];
    
    delta2 =0.5*sigma2_guess ;
    Cons = Cons + [ -delta1 <= Sigma2-sigma2 <= delta2 ];
    %---Convergence constraint
%     if Index > 1
%         coef = 0.95;
%         delta2 = coef*abs(Xk-Xk1);
%         Cons = Cons + [ -delta2 <= X-Xk <= delta2 ];
%     end

    %---Terminal constraints
    Cons = Cons + [ X(1,N) == xf ];
    Cons = Cons + [ X(2,N) == zf ];
    Cons = Cons + [ X(3,N) == vxf ];
    Cons = Cons + [ X(4,N) == vzf ];
    
    %------------------------- Solve the problem --------------------------
    tic
     %options = sdpsettings('verbose',0,'solver','sedumi');
     options = sdpsettings('verbose',0,'solver','mosek');
     %options = sdpsettings('verbose',0,'solver','ecos','ecos.maxit',150);
     %options = sdpsettings('verbose',0,'solver','ecos');
    %options = sdpsettings('verbose',0,'solver','quadprogbb');
     %options = sdpsettings('verbose',0,'solver','sdpt3');
    
    sol = optimize(Cons, Objective, options);
    sol.info
    CPU_time(Index) = toc;
    
    % Check convergence condition and update
    x_opt  = value(X(1,:))';
    z_opt  = value(X(2,:))';
    vx_opt = value(X(3,:))';
    vz_opt = value(X(4,:))';
    u1_opt = value(U(1,:))';
    u2_opt = value(U(2,:))';
    u3_opt = value(U(3,:))';
    sigma_opt = value(Sigma)';
    sigma2_opt = value(Sigma2)';
    
    State_all(:,Index+1) = [x_opt; z_opt; vx_opt; vz_opt];
    Control_all(:,Index) = [u1_opt; u2_opt; u3_opt];
    Sigma_all(:,Index+1) = sigma_opt;
    Sigma2_all(:,Index+1) = sigma2_opt;
    
    del = zeros(6,1);
    del(1) = max(abs(x_opt-x));
    del(2) = max(abs(z_opt-z));
    del(3) = max(abs(vx_opt-vx));
    del(4) = max(abs(vz_opt-vz));
    del(5) = max(abs(sigma_opt-sigma));
    del(6) = max(abs(sigma2_opt-sigma2));
    Del_state(:,Index) = del;
    
    Index
    max_error         = del               % output error between two steps
    Conv_I(Index)     = Index;            % record number of steps
    Conv_x(Index)     = del(1);
    Conv_z(Index)     = del(2);
    Conv_vx(Index)    = del(3);
    Conv_vz(Index)    = del(4);
    Conv_sigma(Index)   = del(5);
    Conv_sigma2(Index)   = del(6);
    Obj(Index)        = value(Objective); % record objective for each step
    
    converge = 9e-1;
    if (del(1) <= converge) && (del(2) <= converge) && (del(3) <= converge)&& (del(4) <= converge)&& (del(5) <= converge)&& (del(6) <= converge)
        break;
    else
        Xk1 = [x'; z'; vx'; vz']; % X_k-2
        x     = x_opt;
        z     = z_opt;
        vx    = vx_opt;
        vz    = vz_opt;
        sigma = sigma_opt;
        sigma2 = sigma2_opt;
        u1 = u1_opt;
        u2 = u2_opt;
        u3 = u3_opt;
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Outputs & Plots                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(Conv_I,Conv_x, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta x (m)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_z, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta z (m)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_vx, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta v_x (m/s)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Conv_vz, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('\Delta v_z (m/s)','FontSize',18)
set(gca,'FontSize',16)
% axis([1 14 0 6e9])
grid on

figure
plot(Conv_I,Obj*1e9, '-o', 'markersize', 7, 'linewidth', 2)
xlabel('Iteration number','FontSize',18)
ylabel('Objective value','FontSize',18)
set(gca,'FontSize',16)
grid on

% %------------------------------- States -----------------------------------
x_tf     = State_all(1*N,1:Index+1)'
z_tf     = State_all(2*N,1:Index+1)'
vx_tf    = State_all(3*N,1:Index+1)'
vz_tf    = State_all(4*N,1:Index+1)'

x     = value(X(1,:))';
z     = value(X(2,:))';
vx    = value(X(3,:))';
vz    = value(X(4,:))';
u1    = value(U(1,:))';
u2    = value(U(2,:))';
u3    = value(U(3,:))';
% 
% % z vs. x
figure
plot(x,z,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 20);
ylabel('Altitude (m)', 'FontSize', 20);
set(gca,'FontSize',18);
grid on

% vx vs. x
figure
plot(x,vx,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Along-Track Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% vz vs. x
figure
plot(x,vz,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Vertical Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% v vs. x
figure
plot(x,sqrt(vx.^2+vz.^2),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Speed (m/s)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% T
figure
plot(x(1:N-1),sqrt(u3(1:N-1)),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Thrust (N)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% theta
figure
plot(x(1:N-1),atan(u1(1:N-1)./u2(1:N-1))*180/pi,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Theta (deg)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% u1^2+u2^2-u3
figure
plot(x(1:N-1),u1(1:N-1).^2+u2(1:N-1).^2-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('u1^2+u2^2-u3', 'FontSize', 18);
set(gca,'FontSize',16);
grid on


% u1^2+u2^2-u3
% % figure
% % plot(x(1:N-1),u1(1:N-1).^2+u2(1:N-1).^2-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
% % xlabel('Along-Track Distance (m)', 'FontSize', 18);
% % ylabel('u1^2+u2^2-u3', 'FontSize', 18);
% % set(gca,'FontSize',16);
% % grid on
figure
plot(x(1:N-1),sqrt(u1(1:N-1).^2+u2(1:N-1).^2)-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('u1^2+u2^2-u3', 'FontSize', 18);
set(gca,'FontSize',16);
grid on
%% cold to warm
Colors = zeros(Index,3);
Colors(:,1) = linspace(0,1,Index);
Colors(:,3) = linspace(1,0,Index);
half = round(Index/2);
Colors(1:half,2)=linspace(0,1,half);
Colors(half+1:Index,2)=linspace(1,0,Index-half);

x_all     = zeros(N,Index+1);
z_all     = zeros(N,Index+1);
vx_all    = zeros(N,Index+1);
vz_all    = zeros(N,Index+1);
u1_all    = zeros(N,Index);
u2_all    = zeros(N,Index);
u3_all    = zeros(N,Index);
for i = 1:Index+1
    x_all(:,i)     = State_all(1:N,i);
    z_all(:,i)     = State_all(N+1:2*N,i);
    vx_all(:,i)    = State_all(2*N+1:3*N,i);
    vz_all(:,i)    = State_all(3*N+1:4*N,i);
end
for i = 1:Index
    u1_all(:,i)    = Control_all(1:N,i);
    u2_all(:,i)    = Control_all(N+1:2*N,i);
    u3_all(:,i)    = Control_all(2*N+1:3*N,i);
end

tau = linspace(0,1,col_points+col_points2)';
% x ~ t
figure
for i = 1:Index
    plot(tau/60, x_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, x_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track distance (m)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% z ~ t
figure
for i = 1:Index
    plot(tau/60, z_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, z_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Altitude (m)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% vx ~ t
figure
for i = 1:Index
    plot(tau/60, vx_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, vx_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track airspeed (m/s)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% vx ~ t
figure
for i = 1:Index
    plot(tau/60, vz_all(:,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau/60, vz_all(:,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Vertical airspeed (m/s)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u1 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u1_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u1_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Along-track control component (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u2 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u2_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u2_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Vertical control component (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% u3 ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, u3_all(1:N-1,i), 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, u3_all(1:N-1,end), 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Net thrust (N)','FontSize',18)
set(gca,'Fontsize',16)
grid on

% theta ~ t
figure
for i = 1:Index
    plot(tau(1:N-1)/60, asin(u1_all(1:N-1,i)./u2_all(1:N-1,i))*180/pi, 'Color', Colors(i,:), 'linewidth', 1.5)
    hold on
end
plot(tau(1:N-1)/60, asin(u1_all(1:N-1,i)./u2_all(1:N-1,i))*180/pi, 'r', 'linewidth', 1.5)
xlabel('Time (min)','FontSize',18)
ylabel('Pitch angle (deg)','FontSize',18)
set(gca,'Fontsize',16)
grid on

%% CPU time
figure
bar(CPU_time)
xlabel('Iteration number','FontSize',18)
ylabel('CPU time (s)','FontSize',18)
set(gca,'Fontsize',16)
grid on

CPU_time = sum(CPU_time)

%% Save the results to compare with SCP
% % % tau = linspace(0,1,col_points)';
% % % t1 = value(Sigma)*tau;
% % % t2 = value(Sigma2)*tau1;
% % % t2 = value(Sigma)+t2;
% % % t = [t1;t2];
% % % 
% % % 
% % % tS     = t;
% % % xS     = x;
% % % zS     = z;
% % % vxS    = vx;
% % % vzS    = vz;
% % % u1S    = u1;
% % % u2S    = u2;
% % % u3S    = u3;
% % % TS     = u3;
% % % thetaS = asin(u1./u3)*180/pi;
% % % save data_scp_2.mat tS xS zS vxS vzS u1S u2S u3S TS thetaS

