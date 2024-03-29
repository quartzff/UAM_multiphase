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

global m rho Sx Sz g CD step
global cf lambda N 
% Parameters
m      = 240;           % vehicle's mass, kg
rho    = 1.225;         % air density, kg/m^3
Sx     = 2.11;          % vehicle's frontal area, m^2
Sz     = 1.47;          % drag coefficient
g      = 9.81;          % gravitaional acceleration, m/s^2
CD     = 1;             % drag coefficient

cf     = 0.5; % contraction factor (for backtracking line-search)
lambda = 0.4; % 0 < eta < 0.5 (for Goldstein Conditions)


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

x1 = 10000;         % phase 1 along-track distance, m
z1 = 500;       % phase 1 altitude, m
vx1 = 0;    % phase 1 along-track airspeed, m/s
vz1 = 0;        % phase 1 vertical airspeed, m/s

xf = 20000;     % final along-track distance, m
zf = 0;         % final altitude, m
vxf = 0;        % final along-track airspeed, m/s
vzf = 0;        % final vertical airspeed, m/s

vmax   = vx0;    % maximum airspeed
Tmin   = 0;      % minimum net thrust
Tmax   = 4800;   % maximum net thrust, N
thetamin = -6*pi/180;    % minimum rotor tip-path-plane pitch angle
thetamax = 6*pi/180;   % maximum rotor tip-path-plane pitch angle

t0 = 0;
tf = 25*60;     % required time of arrival (RTA), min







%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Modeling & Optimization                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Max_iter = 100;   % Maximum number of iteration
col_points = 100;
col_points2 = 150;
tau = linspace(0,1,col_points)';
tau1 = linspace(0,1,col_points2)';
step = tau(2)-tau(1);
N = length(tau)+length(tau1);   % N nodes
sigma_guess = 730; 

sigma2_guess = 1500-730;

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
%Sigma=1440;
% Store the states & controls
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
u1 = ones(N,1)*210;
u2 = ones(N,1)*2300;
u3 = ones(N,1)*2300;

Xk=[x';z';vx';vz'];
Uk = [u1';u2';u3'];

%--------------------------- Iteration procedure --------------------------
for Index = 1:Max_iter
    
    Cons = [];
    % Recursively compute each state (A and B are constant for each step)
    J = 0;
    for i = 1:N-1
       % Cons = Cons + [  Sigma == 1500 ];
        % From last step
         
        x01  = Xk(1,i);
        z01  = Xk(2,i);
        vx01 = Xk(3,i);
        vz01 = Xk(4,i);

        x02  = Xk(1,i+1);
        z02  = Xk(2,i+1);
        vx02 = Xk(3,i+1);
        vz02 = Xk(4,i+1);
        
        
        u1_01 = Uk(1,i);
        u2_01 = Uk(2,i);
        u3_01 = Uk(3,i);
        
        u1_02 = Uk(1,i+1);
        u2_02 = Uk(2,i+1);
        u3_02 = Uk(3,i+1);
        
        % A_i, B_i, b_i
        A1 = zeros(4,4);
        A1(1,3) = 1;
        A1(2,4) = 1;
        A1(3,3) = -rho*vx01*CD*Sx/m;
        A1(4,4) = -rho*vz01*CD*Sz/m;

        B1 = zeros(4,3);
        B1(3,1) = 1/m;
        B1(4,2) = 1/m;
        
        phase_node = col_points;
        if i <=  phase_node
        f1 = [vx01; ...
            vz01; ...
            -rho*vx01^2*CD*Sx/(2*m); ...
            -rho*vz01^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_01);(1/m*u2_01)];
        
        b1 = Sigma*f1 - sigma*A1*[x01; z01; vx01; vz01] - sigma*B1*[u1_01;u2_01;0];

        % A_i+1, B_i+1, b_i+1
        A2 = zeros(4,4);
        A2(1,3) = 1;
        A2(2,4) = 1;
        A2(3,3) = -rho*vx02*CD*Sx/m;
        A2(4,4) = -rho*vz02*CD*Sz/m;

        B2 = zeros(4,3);
        B2(3,1) = 1/m;
        B2(4,2) = 1/m;

        f2 =[vx02; ...
            vz02; ...
            -rho*vx02^2*CD*Sx/(2*m); ...
            -rho*vz02^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_02);(1/m*u2_02)];
        
        b2 = Sigma*f2 - (sigma*A2*[x02; z02; vx02; vz02] + sigma*B2*[ u1_02;u2_02;0]);

        % H1, H2, G1, G2
        H1 = eye(4)+0.5*step*A1*sigma;
        H2 = eye(4)-0.5*step*A2*sigma;
        G1 = 0.5*step*B1*sigma;
        G2 = 0.5*step*B2*sigma;
        
        G3 = 0.5*step*Sigma*B1*[u1_01;u2_01;0];
        G4 = 0.5*step*Sigma*B2*[u1_02;u2_02;0];

        
        %---Linearized dynamics
        Cons = Cons + [ -H1*X(:,i) + H2*X(:,i+1) - G1*U(:,i) - G2*U(:,i+1) == 0.5*step*(b1+b2) ];  % N
%         X(:,i+1) = inv(H2) * (H1*X(:,i) + G1*U(:,i) + G2*U(:,i+1) + 0.5*step*(b1+b2));


        J = J + (1/1e9*step*(sigma*u3_01+sigma*(U(3,i)-u3_01)+u3_01*(Sigma-sigma)));
        else 
        f1 = [vx01; ...
            vz01; ...
            -rho*vx01^2*CD*Sx/(2*m); ...
            -rho*vz01^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_01);(1/m*u2_01)];
        
        b1 = Sigma2*f1 - sigma2*A1*[x01; z01; vx01; vz01] - sigma2*B1*[u1_01;u2_01;0];

        % A_i+1, B_i+1, b_i+1
        A2 = zeros(4,4);
        A2(1,3) = 1;
        A2(2,4) = 1;
        A2(3,3) = -rho*vx02*CD*Sx/m;
        A2(4,4) = -rho*vz02*CD*Sz/m;

        B2 = zeros(4,3);
        B2(3,1) = 1/m;
        B2(4,2) = 1/m;

        f2 =[vx02; ...
            vz02; ...
            -rho*vx02^2*CD*Sx/(2*m); ...
            -rho*vz02^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_02);(1/m*u2_02)];
        
        b2 = Sigma2*f2 - (sigma2*A2*[x02; z02; vx02; vz02] + sigma2*B2*[ u1_02;u2_02;0]);

        % H1, H2, G1, G2
        H1 = eye(4)+0.5*step*A1*sigma2;
        H2 = eye(4)-0.5*step*A2*sigma2;
        G1 = 0.5*step*B1*sigma2;
        G2 = 0.5*step*B2*sigma2;
        
        G3 = 0.5*step*Sigma2*B1*[u1_01;u2_01;0];
        G4 = 0.5*step*Sigma2*B2*[u1_02;u2_02;0];

        
        %---Linearized dynamics
        Cons = Cons + [ -H1*X(:,i) + H2*X(:,i+1) - G1*U(:,i) - G2*U(:,i+1) == 0.5*step*(b1+b2) ];  % N
        J = J + (1/1e9*step*(sigma2*u3_01+sigma2*(U(3,i)-u3_01)+u3_01*(Sigma2-sigma2)));
        end
        %---State constraints
        Cons = Cons + [ x0 <= X(1,i+1) <= xf ];
        Cons = Cons + [ zf <= X(2,i+1) <= z0 ];
        Cons = Cons + [ 0  <= X(3,i+1) <= vmax ];
        Cons = Cons + [ X(1, phase_node) == 10000 ];
        Cons = Cons + [ X(2, phase_node) == z0 ];
        Cons = Cons + [ X(1, phase_node +1) == 10000 ];
        Cons = Cons + [ X(2, phase_node +1) == z0 ];
        Cons = Cons + [ -vmax <= X(4,i+1) <= 0 ];
        Cons = Cons + [ sqrt(X(3,i+1)^2 + X(4,i+1)^2) <= vmax ];
%         Cons = Cons + [ X(3,i+1)^2 + X(4,i+1)^2 <= vmax^2 ];
        
        %---Objective
        
        %J = J + 1/1e9*step*(sigma*(U(3,i)-u3_01)+u3_01*(Sigma-sigma));
        
    end
    
    %---Objective function(functional)
    Objective = J;
    
    %---Control constraints
    for j = 1:N-1
        Cons = Cons + [ U(2,j)*tan(-thetamax) <= U(1,j) <= U(2,j)*tan(thetamax) ];
        Cons = Cons + [ 0 <= U(3,j) <= Tmax^2 ];
        Cons = Cons + [ U(1,j)^2 + U(2,j)^2 <= U(3,j) ];
    end
    Cons = Cons + [  Sigma + Sigma2 == 1500 ];% added time constraint
    Cons = Cons + [ 359 <= Sigma <= 1482 ];% added time constraint
    Cons = Cons + [ 30 <= Sigma2 <= 1400 ];% added time constraint
    
    %---Trust-region constraint
    % |X-Xk|<delta
    Xk = [x'; z'; vx'; vz']; % X_k-1
    delta = [2e3*ones(1,N); 5e2*ones(1,N); 50*ones(1,N); 50*ones(1,N)];
    Cons = Cons + [ -delta <= X-Xk <= delta ];
    
    sigmak = sigma;
    delta1 =0.5*sigma_guess ;
    Cons = Cons + [ -delta1 <= Sigma-sigmak <= delta1 ];
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
     %options = sdpsettings('verbose',0,'solver','mosek');
     options = sdpsettings('verbose',0,'solver','ecos');
    %options = sdpsettings('verbose',0,'solver','quadprogbb');
     %options = sdpsettings('verbose',0,'solver','sdpt3');
    
    sol = optimize(Cons, Objective, options);
    sol.info
    CPU_time(Index) = toc;
    
    X_opt = value(X);
    U_opt = value(U);
    
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
    
    
        %assign state vector and solution pair zk
    Xk1 = Xk;
    Uk1 = Uk;
    sig1 = ones(col_points,1)*sigma;%change 1x1 variable to 1xn vector
    sig1_2 = ones(col_points2,1)*sigma2;
    Zk1 = [Xk1; Uk1; sig1';sig1_2'];%solution pair from last iteration
    
    Uk2 = U_opt;
    Xk2 = X_opt;
    sig2 = ones(col_points,1)*sigma_opt;
    sig2_2 = ones(col_points2,1)*sigma2_opt;
    Zk2 = [Xk2; Uk2; sig2';sig2_2'];%solution pair from new iteration
    c = 1;
    mu=10;% penalty parameter (for constraint violations)
    if Index >= 2
        %c = DampingCoef(Zk1, Zk2, mu)%
        c=0.9;
    end
    DampCoef(Index,1) = c;
    
    Xk2   = Xk1 + c*(Xk2 - Xk1); % updated X_k, 7*N
    Uk2   = Uk1 + c*(Uk2 - Uk1); % updated U_k, 1*N
%     etak2 = etak1 + c*(etak2 - etak1); % updated eta_k, 1*N
    sig2   = sig1 + c*(sig2 - sig1); % updated s_k, 1*N
    sig2_2 = sig1_2 + c*(sig2_2 - sig1_2);
    Zk2   = [Xk2; Uk2; sig2';sig2_2']; % updated Z_k, 8*N
    
    
    
    if (del(1) <= 0.5) && (del(2) <= 0.5) && (del(3) <= 0.5)&& (del(4) <= 0.5)&& (del(5) <= 0.5)&& (del(6) <= 0.5)
        break;
    else
        Xk = Xk2; % X_k-2
        sigma = sigma_opt
        sigma2 = sigma2_opt
        Uk = Uk2;
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
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Altitude (m)', 'FontSize', 18);
set(gca,'FontSize',16);
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

% u1^2+u2^2-u3^2
figure
plot(x(1:N-1),u1(1:N-1).^2+u2(1:N-1).^2-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
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

tau = linspace(0,1,250)';
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
% tS     = t;
% xS     = x;
% zS     = z;
% vxS    = vx;
% vzS    = vz;
% u1S    = u1;
% u2S    = u2;
% u3S    = u3;
% TS     = u3;
% thetaS = asin(u1./u3)*180/pi;
% save data_scp.mat tS xS zS vxS vzS u1S u2S u3S TS thetaS

