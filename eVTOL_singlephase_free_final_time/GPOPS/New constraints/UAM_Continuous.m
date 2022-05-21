function phaseout = UAM_Continuous(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
t     = input.phase.time;
x     = input.phase.state(:,1);
z     = input.phase.state(:,2);
vx    = input.phase.state(:,3);
vz    = input.phase.state(:,4);
u1    = input.phase.control(:,1);
u2    = input.phase.control(:,2);
u3    = input.phase.control(:,3);

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
Tmin = input.auxdata.Tmin;     % minimum net thrust
Tmax = input.auxdata.Tmax;   % maximum net thrust
thetamin = input.auxdata.thetamin;    % minimum rotor tip-path-plane pitch angle
thetamax = input.auxdata.thetamax;

Dx = rho*vx.^2*CD*Sx/2;
Dz = rho*vz.^2*CD*Sz/2;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%
xdot = vx;
zdot = vz;
vxdot = u1/m - Dx/m;
vzdot = u2/m - Dz/m - g;
phaseout.dynamics  = [xdot, zdot, vxdot, vzdot];

% ---------------------------------------------------%
% ----------- Evaluate Path Constraints ------------ %
% ---------------------------------------------------%
path1 = sqrt(vx.^2+vz.^2)-vmax;
path2 = u2*tan(thetamin)-u1;
path3 = u1-u2*tan(thetamax)-u1;
path4 = u1.^2+u2.^2-u3;
phaseout.path  = [path1, path2, path3, path4];
  
% ---------------------------------------------------%
% ------------ Evaluate Integral Cost -------------- %
% ---------------------------------------------------%
phaseout.integrand = 1/2*u3/1e9; % minimum energy
