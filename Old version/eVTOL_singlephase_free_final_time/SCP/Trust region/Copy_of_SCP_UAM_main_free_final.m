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
global  N 


% trust-region parameters
mu    = 100; % very good using Boyd's method, close to GPOPS's solution
c     = 0.3;
beta1 = 1.1;
beta2 = 0.5;


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
vx0 = 13.85; % initial along-track airspeed, m/s
vz0 = 0;        % initial vertical airspeed, m/s

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
%tf = 25*60;     % required time of arrival (RTA), min

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Modeling & Optimization                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Max_iter = 30;   % Maximum number of iteration
col_points = 60;
tau = linspace(0,1,col_points)';
step = tau(2)-tau(1);

N = length(tau);   % N nodes


%set initial state variables
x = linspace(x0,xf,col_points)';
z = linspace(z0,zf,col_points)';
vx = linspace(vx0,vxf,col_points)';
vz = linspace(vz0,vzf,col_points)';

X = sdpvar(4,N);
U = sdpvar(3,N);
Sigma = sdpvar(1,1);

sigma_guess = 1470;%1460; % guess for the final time
% Store the states & controls
State_all = zeros(4*N, Max_iter+1);
State_all(:,1) = [x; z; vx; vz];
Control_all = zeros(3*N, Max_iter);
Del_state = zeros(5, Max_iter);
Sigma_all = zeros(1, Max_iter+1);
Sigma_all(:,1) = sigma_guess;
%X(:,1) = [x0; z0; vx0; vz0];
% U(:,1) = [0; 0; 0];


sigma = sigma_guess;%sigma for the last iteration
u1 = ones(length(tau),1)*210;%initialize all control variables
u2 = ones(length(tau),1)*2300;
u3 = ones(length(tau),1)*2300;
%define control and state vectors
Xk=[x';z';vx';vz'];
Uk = [u1';u2';u3'];


% initialize the trust-region radius
% delta = [xf/100*ones(1,N); zf/100*ones(1,N); vmax/10*ones(1,N);...
%     vmax/10*ones(1,N)];

%--------------------------- Iteration procedure --------------------------
for Index = 1:Max_iter
    
    Cons = [];
    % Recursively compute each state (A and B are constant for each step)
    J = 0;
    for i = 1:N-1
        
        % From last step
        x01  = Xk(1,i);z01  = Xk(2,i);vx01 = Xk(3,i);vz01 = Xk(4,i);
        x02  = Xk(1,i+1);z02  = Xk(2,i+1);vx02 = Xk(3,i+1);vz02 = Xk(4,i+1);
        
        u1_01 = Uk(1,i);u2_01 = Uk(2,i);u3_01 = Uk(3,i);
        u1_02 = Uk(1,i+1);u2_02 = Uk(2,i+1);u3_02 = Uk(3,i+1);
       
        A1 = zeros(4,4);
        A1(1,3) = 1;
        A1(2,4) = 1;
        A1(3,3) = -rho*vx01*CD*Sx/m;
        A1(4,4) = -rho*vz01*CD*Sz/m;

        B1 = zeros(4,3);
        B1(3,1) = 1/m;
        B1(4,2) = 1/m;

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

        u11 = U(:,i);
        %---Linearized dynamics
        Cons = Cons + [ -H1*X(:,i) + H2*X(:,i+1) - G1*U(:,i) - G2*U(:,i+1) == 0.5*step*(b1+b2) ];  % N

        
        %---State constraints
        Cons = Cons + [ x0 <= X(1,i+1) <= xf ];
        Cons = Cons + [ zf <= X(2,i+1) <= z0 ];
        Cons = Cons + [ 0  <= X(3,i+1) <= vmax ];
        Cons = Cons + [ -vmax <= X(4,i+1) <= 0 ];
        Cons = Cons + [ sqrt(X(3,i+1)^2 + X(4,i+1)^2) <= vmax ];
        
        %---State constraints
        Cons = Cons + [ U(3,i+1)*sin(thetamin) <= U(1,i+1) <= U(3,i+1)*sin(thetamax) ];
        Cons = Cons + [ U(3,i+1)*cos(thetamax) <= U(2,i+1) <= U(3,i+1)*cos(0) ];
        Cons = Cons + [ 0 <= U(3,i+1) <= Tmax ];
        Cons = Cons + [ sqrt(U(1,i+1)^2 + U(2,i+1)^2) <= U(3,i+1) ];
        %---Objective
        %J = J + 1/1e9*step*1/2*U(3,i+1)^2*Sigma;
        J = J + (1/1e9*step*(sigma*u3_01+sigma*(U(3,i)-u3_01)+u3_01*(Sigma-sigma)));
         %J = J + 1/1e9 * step*((1/2*u3_01^2*sigma)+1/2*u3_01^2*(Sigma - sigma)+u3_01*sigma*(U(3,i+1)-u3_01));%First order linearization
        %J = J + 1/1e9 * step * 1/2*U(3,i)^2*sigma; %Psedo-linearization
  
    end
    
    %---Objective function(functional)
    Objective = J;
    
        %---Control constraints
    for j = 1:N
        Cons = Cons + [ U(3,j)*sin(thetamin) <= U(1,j) <= U(3,j)*sin(thetamax) ];
        Cons = Cons + [ U(3,j)*cos(thetamax) <= U(2,j) <= U(3,j)*cos(0) ];
        Cons = Cons + [ 0 <= U(3,j) <= Tmax ];
        Cons = Cons + [ sqrt(U(1,j)^2 + U(2,j)^2) <= U(3,j) ];
        
        

    end
    
     %Cons = Cons + [  Sigma == 1480.63 ];% added time constraint
    %Cons = Cons + [  Sigma == 912.2 ];
    Cons = Cons + [ 400 <= Sigma <= 1600 ];% added time constraint
    
    %---Trust-region constraint
    % |X-Xk|<delta
    %Xk = [x'; z'; vx'; vz']; % X_k-1
    delta = [1000*ones(1,N); 50*ones(1,N); 50*ones(1,N); 50*ones(1,N)];
    Cons = Cons + [ -delta <= X-Xk <= delta ];
    
     
     delta1 =0.1*sigma_guess ;
     Cons = Cons + [ -delta1 <= Sigma-sigma <= delta1 ];


    %---Terminal constraints
    Cons = Cons + [ X(1,N) == xf ];
    Cons = Cons + [ X(2,N) == zf ];
    Cons = Cons + [ X(3,N) == vxf ];
    Cons = Cons + [ X(4,N) == vzf ];
    
    %------------------------- Solve the problem --------------------------
    tic
     %options = sdpsettings('verbose',0,'solver','sedumi');
     options = sdpsettings('verbose',0,'solver','mosek');
    
    sol = optimize(Cons, Objective, options)
    sol.info
    CPU_time(Index) = toc;
    
    X_opt = value(X);
    U_opt = value(U);
    % Check convergence condition and update
    x_opt  = X_opt(1,:)';
    z_opt  = X_opt(2,:)';
    vx_opt = X_opt(3,:)';
    vz_opt = X_opt(4,:)';
    u1_opt = U_opt(1,:)';
    u2_opt = U_opt(2,:)';
    u3_opt = U_opt(3,:)';
    sigma_opt = value(Sigma)';
    
     
    State_all(:,Index+1) = [x_opt; z_opt; vx_opt; vz_opt];
    Control_all(:,Index) = [u1_opt; u2_opt; u3_opt];
    Sigma_all(:,Index+1) = sigma_opt;
    
    
    del = zeros(5,1);
    del(1) = max(abs(x_opt-Xk(1,:)'));
    del(2) = max(abs(z_opt-Xk(2,:)'));
    del(3) = max(abs(vx_opt-Xk(3,:)'));
    del(4) = max(abs(vz_opt-Xk(4,:)'));
    del(5) = max(abs(sigma_opt-sigma));
    Del_state(:,Index) = del;
    
    Index
    max_error         = del               % output error between two steps
    Conv_I(Index)     = Index;            % record number of steps
    Conv_x(Index)     = del(1);
    Conv_z(Index)     = del(2);
    Conv_vx(Index)    = del(3);
    Conv_vz(Index)    = del(4);
    Conv_sigma(Index)   = del(5);
    Obj(Index)        = value(Objective); % record objective for each step
    
    %assign state vector and solution pair zk
% % % % %     Xk1 = Xk;
% % % % %     Uk1 = Uk;
% % % % %     sig1 = ones(col_points,1)*sigma;%change 1x1 variable to 1xn vector
% % % % %     Zk1 = [Xk1; Uk1; sig1'];%solution pair from last iteration
% % % % %     
% % % % %     Uk2 = U_opt;
% % % % %     Xk2 = X_opt;
% % % % %     sig2 = ones(col_points,1)*sigma_opt;
% % % % %     Zk2 = [Xk2; Uk2; sig2'];%solution pair from new iteration
% % % % %     c = 1;
% % % % %     mu=10;% penalty parameter (for constraint violations)
% % % % %     if Index >= 2
% % % % %         c = DampingCoef(Zk1, Zk2, mu)
% % % % %     end
% % % % %     DampCoef(Index,1) = c;
% % % % %     
% % % % %     Xk2   = Xk1 + c*(Xk2 - Xk1); % updated X_k, 7*N
% % % % %     Uk2   = Uk1 + c*(Uk2 - Uk1); % updated U_k, 1*N
% % % % % %     etak2 = etak1 + c*(etak2 - etak1); % updated eta_k, 1*N
% % % % %     sig2   = sig1 + c*(sig2 - sig1); % updated s_k, 1*N
% % % % %     Zk2   = [Xk2; Uk2; sig2']; % updated Z_k, 8*N
    

    %DampingCoef(Zk1, Zk2, mu)
    if (del(1) <= 0.5) && (del(2) <= 0.8) && (del(3) <= 0.5)&& (del(4) <= 0.5)&& (del(5) <= 2)
        break;
    else
        Xk = X_opt; % X_k-2
        sigma = sigma_opt
        Uk = U_opt;

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
plot(x(1:N-1),u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Thrust (N)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% theta
figure
plot(x(1:N-1),asin(u1(1:N-1)./u3(1:N-1))*180/pi,'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('Theta (deg)', 'FontSize', 18);
set(gca,'FontSize',16);
grid on

% u1^2+u2^2-u3^2
figure
plot(x(1:N-1),sqrt(u1(1:N-1).^2+u2(1:N-1).^2)-u3(1:N-1),'-o', 'markersize', 7, 'linewidth', 1.5);
xlabel('Along-Track Distance (m)', 'FontSize', 18);
ylabel('u1^2+u2^2-u3^2', 'FontSize', 18);
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
% t=value(Sigma)*tau;
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

