% This program is used to solve UAM arrival and descent trajectory optimization

close all
clear
clc
format long
tic
%-------------------------------------------------------------------------%
%------------------ Provide Auxiliary Data for Problem -------------------%
%-------------------------------------------------------------------------%
auxdata.m      = 240;           % vehicle's mass, kg
auxdata.rho    = 1.225;          % air density, kg/m^3
auxdata.Sx     = 2.11;            % vehicle's frontal area, m^2
auxdata.Sz     = 1.47;           % drag coefficient
auxdata.g      = 9.81;            % gravitaional acceleration, m/s^2
auxdata.CD     = 1;          % drag coefficient

%-------------------------------------------------------------------%
%----------------------- Boundary Conditions -----------------------%
%-------------------------------------------------------------------%

x0 = 0;         % initial along-track distance, m
z0 = 500;       % initial altitude, m
vx0 = 27.78;    % initial along-track airspeed, m/s
vz0 = 0;        % initial vertical airspeed, m/s

xf = 20000;     % final along-track distance, m
zf = 0;         % final altitude, m
vxf = 0;        % final along-track airspeed, m/s
vzf = 0;        % final vertical airspeed, m/s

t0 = 0;
t1_low = xf/vx0;  % Minimum cruise time
%t1_up = (25*60)-(500/11); % Max cruise time
t1_up = (25*60)-(500/27.78);
tf = 25*60;     % required time of arrival (RTA), min

t1min = xf/vx0;    % Minimum time to arrive fixed point
t1max = tf - (500/2.5); % Max time to arrive fixed point

auxdata.xmin   = x0;              % minimum x, m
auxdata.xmax   = xf;           % maximum x, m
auxdata.zmin   = zf;              % minimum z, m
auxdata.zmax   = z0;           % maximum z, m
auxdata.vmin   = 0;             % minimum Vx, m/s
auxdata.vmax   = abs(vx0);         % maximum Vx, m/s
auxdata.Tmin   = 0;     % minimum net thrust
auxdata.Tmax   = 4800;   % maximum net thrust
auxdata.thetamin = -6*pi/180;    % minimum rotor tip-path-plane pitch angle
auxdata.thetamax = 6*pi/180;   % maximum rotor tip-path-plane pitch angle

%-------------------------------------------------------------------%
%----------------------- Limits on Variables -----------------------%
%-------------------------------------------------------------------%
xMin  = auxdata.xmin;              xMax  = auxdata.xmax;
zMin  = auxdata.zmin;              zMax  = auxdata.zmax;
vxMin = auxdata.vmin;             vxMax  = auxdata.vmax;
vzMin = -auxdata.vmax;             vzMax  = auxdata.vmin;
u1Min = auxdata.Tmax*sin(auxdata.thetamin);
u1Max = auxdata.Tmax*sin(auxdata.thetamax);
u2Min = auxdata.Tmin*cos(auxdata.thetamax);
u2Max = auxdata.Tmax*cos(0);
u3Min = auxdata.Tmin;
u3Max = auxdata.Tmax;
%-------------------------------------------------------------------%
%--------------- Set Up Problem Using Data Provided Above ----------%
%-------------------------------------------------------------------%


% -----------------Guess values-------------------------------------%


iphase = 1;

bounds.phase(iphase).initialtime.lower = [t0];
bounds.phase(iphase).initialtime.upper = [t0];
bounds.phase(iphase).finaltime.lower = [t1_low];
bounds.phase(iphase).finaltime.upper = [t1_up];

bounds.phase(iphase).initialstate.lower = [x0, z0, vx0, vz0];
bounds.phase(iphase).initialstate.upper = [x0, z0, vx0, vz0];
bounds.phase(iphase).state.lower = [xMin, zMax, vxMin, vzMin];
bounds.phase(iphase).state.upper = [xMax, zMax, vxMax, vzMax];
bounds.phase(iphase).finalstate.lower = [xf, z0, vxf, vzf];
bounds.phase(iphase).finalstate.upper = [xf, z0, vxf, vzf];
bounds.phase(iphase).control.lower = [u1Min, u2Min, u3Min];
bounds.phase(iphase).control.upper = [u1Max, u2Max, u3Max];
bounds.phase(iphase).path.lower = [-1e9, -1e9, -1e9, -1e9];
bounds.phase(iphase).path.upper = [0, 0, 0, 0];

guess.phase(iphase).time = [t0; t1_low];
guess.phase(iphase).state(:,1) = [x0; xf];
guess.phase(iphase).state(:,2) = [z0; z0];
guess.phase(iphase).state(:,3) = [vx0; vxf];
guess.phase(iphase).state(:,4) = [vz0; vzf];
guess.phase(iphase).control(:,1) = [u1Min; u1Max];
guess.phase(iphase).control(:,2) = [u2Min; u2Max];
guess.phase(iphase).control(:,3) = [u3Min; u3Max];
guess.phase(iphase).integral = 0;
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = 1e15;



iphase = 2;
bounds.phase(iphase).initialtime.lower = [t1_low];
bounds.phase(iphase).initialtime.upper = [t1_up];
bounds.phase(iphase).finaltime.lower = [tf];
bounds.phase(iphase).finaltime.upper = [tf];

bounds.phase(iphase).initialstate.lower = [xf, z0, vxf, vz0];
bounds.phase(iphase).initialstate.upper = [xf, z0, vxf, vz0];
bounds.phase(iphase).state.lower = [xMax, zMin, vxMin, vzMin];
bounds.phase(iphase).state.upper = [xMax, zMax, vxMax, vzMax];
bounds.phase(iphase).finalstate.lower = [xf, zf, vxf, vzf];
bounds.phase(iphase).finalstate.upper = [xf, zf, vxf, vzf];
bounds.phase(iphase).control.lower = [u1Min, u2Min, u3Min];
bounds.phase(iphase).control.upper = [u1Max, u2Max, u3Max];
bounds.phase(iphase).path.lower = [-1e9, -1e9, -1e9, -1e9,];
bounds.phase(iphase).path.upper = [0, 0, 0, 0];

guess.phase(iphase).time = [t1_low; tf];
guess.phase(iphase).state(:,1) = [xf; xf];
guess.phase(iphase).state(:,2) = [z0; zf];
guess.phase(iphase).state(:,3) = [vxf; vxf];
guess.phase(iphase).state(:,4) = [vz0; vzf];
guess.phase(iphase).control(:,1) = [u1Min; u1Max];
guess.phase(iphase).control(:,2) = [u2Min; u2Max];
guess.phase(iphase).control(:,3) = [u3Min; u3Max];
guess.phase(iphase).integral = 0;
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = 1e15;



%-------------------------------------------------------------------------%
%------------- Set up Event Constraints That Link Phases -----------------%
%-------------------------------------------------------------------------%
bounds.eventgroup(1).lower = [zeros(1,4), 0];
bounds.eventgroup(1).upper = [zeros(1,4), 0];

% Terminal constraint
bounds.eventgroup(2).lower = [xf, zf, vxf, vzf,  tf];%%
bounds.eventgroup(2).upper = [xf, zf, vxf, vzf,  tf];
% *BOLD TEXT*

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method       = 'hp-PattersonRao';
mesh.maxiterations = 30;
mesh.colpointsmin = 3;
mesh.colpointsmax = 10;
% mesh.tolerance    = 1e-6;
mesh.tolerance    = 1e-3;

%-------------------------------------------------------------------%
%---------- Configure Setup Using the information provided ---------%
%-------------------------------------------------------------------%
setup.name                           = 'Verify-CAV-Problem';
setup.functions.continuous           = @UAM_Continuous;
setup.functions.endpoint             = @UAM_Endpoint;
setup.auxdata                        = auxdata;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.displaylevel                   = 1; % 1-display off; 2-display on
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'mumps';
setup.derivatives.supplier           = 'sparseCD';
setup.derivatives.dependencies       = 'sparseNaN';
setup.derivatives.derivativelevel    = 'second';
setup.scales.method                  = 'automatic-bounds';
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
J = output.result.objective*1e9
toc
