function phaseout = UAV_Continuous(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%

t          = input.phase.time;
x     = input.phase.state(:,1);
y     = input.phase.state(:,2);
z     = input.phase.state(:,3);
phi   = input.phase.state(:,4);
theta = input.phase.state(:,5);
psi   = input.phase.state(:,6);
xd     = input.phase.state(:,7);
yd     = input.phase.state(:,8);
zd     = input.phase.state(:,9);
phid   = input.phase.state(:,10);
thetad = input.phase.state(:,11);
psid   = input.phase.state(:,12);



n1  = input.phase.control(:,1);
n2  = input.phase.control(:,2);
n3  = input.phase.control(:,3);
n4  = input.phase.control(:,4);


%control vector
u1 = n1;
u2 = n2;
u3 = n3;
u4 = n4;


% ---------------------------------------------------%
% ------- Compute the Aerodynamic Quantities --------%
% ---------------------------------------------------%
m = input.auxdata.m;
Ix = input.auxdata.Ix;
Iy = input.auxdata.Iy;
Iz = input.auxdata.Iz;
g = input.auxdata.g;







%other interia constant
a1 = (Iy-Iz)/Ix;
a2 = (Iz-Ix)/Iy;
a3 = (Ix-Iy)/Iz;
b1 = 1/Ix;
b2 = 1/Iy;
b3 = 1/Iz;




% ---------------------------------------------------%
% ---- Flight dynamic ODE ---- %
% ---------------------------------------------------%
xdot = xd;
ydot = yd;
zdot = zd;
phidot = phid;
thetadot = thetad;
psidot = psid;
xddot = -u1.*(sin(psi).*sin(phi)+cos(psi).*sin(theta).*cos(phi))./m;
yddot = -u1.*(-cos(psi).*sin(phi)+sin(psi).*sin(theta).*cos(phi))./m;
zddot = -u1.*(cos(phi).*cos(theta))./m + g;
psiddot = b1.*u2+a1*phid.*thetad;
thetaddot = b2.*u3+a2*phid.*psid;
phiddot = b3.*u4+a3*thetad.*psid;



phaseout.dynamics  = [xdot,ydot,zdot,psidot,thetadot,phidot,xddot,yddot,zddot,psiddot,thetaddot,phiddot];

phaseout.integrand = u1.*u1;


