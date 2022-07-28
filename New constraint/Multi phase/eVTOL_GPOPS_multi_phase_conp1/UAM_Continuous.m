function phaseout = UAM_Continuous(input)

%---------------------%
% Dynamics in Phase 1 %
%---------------------%


% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
t1     = input.phase(1).time;
x1     = input.phase(1).state(:,1);
z1     = input.phase(1).state(:,2);
vx1    = input.phase(1).state(:,3);
vz1    = input.phase(1).state(:,4);
u1    = input.phase(1).control(:,1);
u2    = input.phase(1).control(:,2);
u3    = input.phase(1).control(:,3);

% ---------------------------------------------------%
% ------- Compute the Aerodynamic Quantities --------%
% ---------------------------------------------------%
m   = input.auxdata.m;           % vehicle's mass, kg
rho = input.auxdata.rho;          % air density, kg/m^3
Sx  = input.auxdata.Sx;            % vehicle's frontal area, m^2
Sz  = input.auxdata.Sz;           % drag coefficient
g   = input.auxdata.g;            % gravitaional acceleration, m/s^2
CD  = input.auxdata.CD;          % drag coefficient
vmax = input.auxdata.vmax;
thetamin = input.auxdata.thetamin;    % minimum rotor tip-path-plane pitch angle
thetamax = input.auxdata.thetamax;

Dx1 = rho*vx1.^2*CD*Sx/2;
Dz1 = rho*vz1.^2*CD*Sz/2;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%
xdot = vx1;
zdot = vz1;
vxdot = u1/m - Dx1/m;
vzdot = u2/m - Dz1/m - g;
phaseout(1).dynamics  = [xdot, zdot, vxdot, vzdot];

% ---------------------------------------------------%
% ----------- Evaluate Path Constraints ------------ %
% ---------------------------------------------------%
path1  = sqrt(vx1.^2+vz1.^2)-vmax;
path2 = u2*tan(thetamin)-u1;
path3 = u1-u2*tan(thetamax)-u1;
path4 = u1.^2+u2.^2-u3;


phaseout(1).path  = [path1, path2, path3, path4];
phaseout(1).integrand = u3/1e9;  
%---------------------%
% Dynamics in Phase 2 %
%---------------------%


% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
t2     = input.phase(2).time;
x2     = input.phase(2).state(:,1);
z2     = input.phase(2).state(:,2);
vx2    = input.phase(2).state(:,3);
vz2    = input.phase(2).state(:,4);
u1_2    = input.phase(2).control(:,1);
u2_2    = input.phase(2).control(:,2);
u3_2    = input.phase(2).control(:,3);
Dx = rho*vx2.^2*CD*Sx/2;
Dz = rho*vz2.^2*CD*Sz/2;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%
xdot2 = vx2;
zdot2 = vz2;
vxdot2 = u1_2/m - Dx/m;
vzdot2 = u2_2/m - Dz/m - g;
phaseout(2).dynamics  = [xdot2, zdot2, vxdot2, vzdot2];

% ---------------------------------------------------%
% ----------- Evaluate Path Constraints ------------ %
% ---------------------------------------------------%
path1_2  = sqrt(vx2.^2+vz2.^2)-vmax;
path2_2 = u2_2*tan(thetamin)-u1_2;
path3_2 = u1_2-u2_2*tan(thetamax)-u1_2;
path4_2 = u1_2.^2+u2_2.^2-u3_2;


phaseout(2).path  = [path1_2, path2_2, path3_2, path4_2];

% ---------------------------------------------------%
% ------------ Evaluate Integral Cost -------------- %
% ---------------------------------------------------%
phaseout(2).integrand = u3_2/1e9; % minimum energy
