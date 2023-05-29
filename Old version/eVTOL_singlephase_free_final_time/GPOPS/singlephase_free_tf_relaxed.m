% This program is used to solve UAM arrival and descent trajectory
% optimization with free final time

close all
clear
clc
format long
tic
%-------------------------------------------------------------------------%
%------------------ Provide Auxiliary Data for Problem -------------------%
%-------------------------------------------------------------------------%
auxdata.m      = 240;           % vehicle's mass, kg
auxdata.rho    = 1.225;         % air density, kg/m^3
auxdata.Sx     = 2.11;          % vehicle's frontal area, m^2
auxdata.Sz     = 1.47;          % drag coefficient
auxdata.g      = 9.81;          % gravitaional acceleration, m/s^2
auxdata.CD     = 1;             % drag coefficient

%-------------------------------------------------------------------%
%----------------------- Boundary Conditions -----------------------%
%-------------------------------------------------------------------%
x0 = 0;         % initial along-track distance, m
z0 = 500;       % initial altitude, m
vx0 = 13.85;    % initial along-track airspeed, m/s
vz0 = 0;        % initial vertical airspeed, m/s

xf = 20000;     % final along-track distance, m
zf = 0;         % final altitude, m
vxf = 0;        % final along-track airspeed, m/s
vzf = 0;        % final vertical airspeed, m/s

t0 = 0;
%tf = 25*60;     % required time of arrival (RTA), min

auxdata.xmin   = x0;           % minimum x, m
auxdata.xmax   = xf;           % maximum x, m
auxdata.zmin   = zf;           % minimum z, m
auxdata.zmax   = z0;           % maximum z, m
auxdata.vmin   = 0;            % minimum Vx, m/s
auxdata.vmax   = vx0;          % maximum Vx, m/s
auxdata.Tmin   = 0;            % minimum net thrust
auxdata.Tmax   = 4800;         % maximum net thrust
auxdata.thetamin = -6*pi/180;  % minimum rotor tip-path-plane pitch angle
auxdata.thetamax = 6*pi/180;   % maximum rotor tip-path-plane pitch angle

%-------------------------------------------------------------------%
%----------------------- Limits on Variables -----------------------%
%-------------------------------------------------------------------%
xMin  = auxdata.xmin;              xMax  = auxdata.xmax;
zMin  = auxdata.zmin;              zMax  = auxdata.zmax;
vxMin = auxdata.vmin;             vxMax  = auxdata.vmax;
vzMin = -auxdata.vmax;            vzMax  = auxdata.vmin;
u1Min = auxdata.Tmax*sin(auxdata.thetamin);
u1Max = auxdata.Tmax*sin(auxdata.thetamax);
u2Min = auxdata.Tmin*cos(auxdata.thetamax);
u2Max = auxdata.Tmax*cos(0);
u3Min = auxdata.Tmin;
u3Max = auxdata.Tmax;

%-------------------------------------------------------------------%
%--------------- Set Up Problem Using Data Provided Above ----------%
%-------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = 300;
bounds.phase.finaltime.upper = 1800;

% initial constraints
bounds.phase.initialstate.lower = [x0, z0, vx0, vz0];
bounds.phase.initialstate.upper = [x0, z0, vx0, vz0];

% terminal constraints
bounds.phase.finalstate.lower = [xf, zf, vxf, vzf];
bounds.phase.finalstate.upper = [xf, zf, vxf, vzf];

% state constraints
bounds.phase.state.lower = [xMin, zMin, vxMin, vzMin];
bounds.phase.state.upper = [xMax, zMax, vxMax, vzMax];

% control constraints
bounds.phase.control.lower = [u1Min, u2Min, u3Min];
bounds.phase.control.upper = [u1Max, u2Max, u3Max];

% path constraints
bounds.phase.path.lower = [-1e9, -1e9, -1e9, -1e9, -1e9, -1e9];
bounds.phase.path.upper = [0, 0, 0, 0, 0, 0];

% integral objective function
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1e15;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
tGuess              = [t0; 1400];
xGuess              = [x0; xf];
zGuess              = [z0; zf];
vxGuess             = [vx0; vxf];
vzGuess             = [vz0; vzf];
u1Guess             = [u1Min; u1Max];
u2Guess             = [u2Min; u2Max];
u3Guess             = [u3Min; u3Max];
guess.phase.state   = [xGuess, zGuess, vxGuess, vzGuess];
guess.phase.control = [u1Guess, u2Guess, u3Guess];
guess.phase.time    = tGuess;
guess.phase.integral = 0;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method       = 'hp-LiuRao-Legendre';%'hp-PattersonRao';
mesh.maxiterations = 25;
mesh.colpointsmin = 3;
mesh.colpointsmax = 15;
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
setup.derivatives.derivativelevel    = 'second';
setup.scales.method                  = 'automatic-bounds';
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
J = output.result.objective*1e9
toc
