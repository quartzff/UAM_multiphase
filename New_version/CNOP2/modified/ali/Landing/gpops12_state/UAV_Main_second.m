% This program is used to solve optimal quadrotor landing control problem
% High-fidelity aero model is supposed to be included
% The code is based on Chapel's drone pollination project

close all
clear all
clearvars
clc
format long
tic
%-------------------------------------------------------------------------%
%------------------ Provide Auxiliary Data for Problem -------------------%
%-------------------------------------------------------------------------%
auxdata.m    = 0.69;%3.6;     % DJI M100 maximum takeoff mass 3.6 kg
auxdata.Ix = 0.0469;
auxdata.Iy = 0.0358;
auxdata.Iz = 0.0673;
auxdata.l = 0.225;%0.650;%0.225; % Diagonal wheelbase: 650 mm
auxdata.g = 9.81;
auxdata.pho = 1.225;% air density
auxdata.D= 0.0762*2;%0.33; % Propoller diameter




%-------------------------------------------------------------------%
%----------------------- Boundary Conditions -----------------------%
%-------------------------------------------------------------------%
xi = -3;
yi = 4;
zi = -5;
% xi = -6;
% yi = 8;
% zi = -10;
phii = 0;
thetai = 0;
psii = 0;
xdi = 0;
ydi = 0;
zdi = 0;
% xdi = 0;
% ydi = 0.5;
% zdi = 0;
phidi = 0;
thetadi = 0;
psidi = 0;

auxdata.xi = xi;
auxdata.yi = yi;
auxdata.zi = zi;
F_zi = 6.9;
tau_xi =0 ; 
tau_yi = 0;
tau_zi = 0;


% final condition
xf = 0;
yf = 0;
zf = 0;
phif = 0;
thetaf = 0;
psif = 0;
xdf = 0;
ydf = 0;
zdf = 0;
phidf = 0;
thetadf = 0;
psidf = 0;
auxdata.xf = xf;
auxdata.yf = yf;
auxdata.zf = zf;

F_zf = 6.9;
tau_xf = 0;
tau_yf = 0;
tau_zf = 0;


t0 = 0;
tf = 4;

%-------------------------------------------------------------------%
%----------------------- Limits on Variables -----------------------%
%-------------------------------------------------------------------%

minx = -2*abs(xi);   maxx = 2*abs(xi);
miny = -2*abs(yi);   maxy = 2*abs(yi);
minz = -2*abs(zi);   maxz = 0;
minphi = -0.5*pi;   maxphi = 0.5*pi;
mintheta = -0.5*pi;   maxtheta = 0.5*pi;
minpsi = -pi;   maxpsi = pi;

minxd = -3;   maxxd = 3;
minyd = -3;   maxyd = 3;
minzd = -3;   maxzd = 3;
minphid = -0.25*pi;   maxphid = 0.25*pi;
minthetad = -0.25*pi;   maxthetad = 0.25*pi;
minpsid = -0.25*pi;   maxpsid = 0.25*pi;

minF_z = 0; maxF_z = 11;


mintau_x = -1;         maxtau_x = 1;
mintau_y = -1;         maxtau_y = 1;
mintau_z = -1;         maxtau_z = 1;




%-------------------------------------------------------------------%
%--------------- Set Up Problem Using Data Provided Above ----------%
%-------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf;
bounds.phase.finaltime.upper = tf;

% initial constraints    psidot,thetadot,phidot,xddot,yddot,zddot,psiddot,thetaddot,phiddot,
bounds.phase.initialstate.lower = [xi,yi,zi,psii,thetai,phii,xdi,ydi,zdi,psidi,thetadi,phidi];
bounds.phase.initialstate.upper = [xi,yi,zi,psii,thetai,phii,xdi,ydi,zdi,psidi,thetadi,phidi];
% terminal constraints
bounds.phase.finalstate.lower = [xf,yf,zf,psif,thetaf,phif,xdf,ydf,zdf,psidf,thetadf,phidf];
bounds.phase.finalstate.upper = [xf,yf,zf,psif,thetaf,phif,xdf,ydf,zdf,psidf,thetadf,phidf];

% state constraints
bounds.phase.state.lower = [minx,miny,minz,minpsi,mintheta,minphi,minxd,minyd,minzd,minpsid,minthetad,minphid];
bounds.phase.state.upper = [maxx,maxy,maxz,maxpsi,maxtheta,maxphi,maxxd,maxyd,maxzd,maxpsid,maxthetad,maxphid];

% control constraints
bounds.phase.control.lower = [minF_z,mintau_x,mintau_y,mintau_z];
bounds.phase.control.upper = [maxF_z,maxtau_x,maxtau_y,maxtau_z];

% integral objective function
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1e9;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
tGuess              = [t0; tf];
xGuess              = [minx; maxx];
yGuess              = [miny; maxy];
zGuess              = [minz; maxz];
phiGuess            = [minphi; maxphi];
thetaGuess          = [mintheta; maxtheta];
psiGuess            = [minpsi; maxpsi];
xdGuess             = [minxd; maxxd];
ydGuess             = [minyd; maxyd];
zdGuess             = [minzd; maxzd];
phidGuess           = [minphid; maxphid];
thetadGuess         = [minthetad; maxthetad];
psidGuess           = [minpsid; maxpsid];

F_zdGuess           = [minF_z; maxF_z];
tau_xdGuess         = [mintau_x; maxtau_x];
tau_ydGuess         = [mintau_y; maxtau_y];
tau_zdGuess         = [mintau_z; maxtau_z];




guess.phase.state   = [xGuess,yGuess,zGuess, psiGuess,thetaGuess,phiGuess,xdGuess,ydGuess,zdGuess,psidGuess,thetadGuess,phidGuess];
guess.phase.control = [F_zdGuess,tau_xdGuess,tau_ydGuess,tau_zdGuess];
guess.phase.time    = tGuess;
guess.phase.integral = 0;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method       = 'hp-PattersonRao';
mesh.maxiterations = 50 ;
mesh.colpointsmin = 3;
mesh.colpointsmax = 15;
% mesh.tolerance    = 1e-6; % Slow if using smaller tolerance
mesh.tolerance    = 1e-3;

%-------------------------------------------------------------------%
%---------- Configure Setup Using the information provided ---------%
%-------------------------------------------------------------------%
setup.name                           = 'Verify-Landing-Problem';
setup.functions.continuous           = @UAV_Continuous;
setup.functions.endpoint             = @UAV_Endpoint;
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
setup.nlp.ipoptoptions.maxiterations = 70;
setup.nlp.ipoptoptions.tolerance = 1e-08;

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
sum_ctrl = output.result.objective
toc


%