% Calculate the value of the merit function
function Phi = MeritFunc(Z, mu)
global m rho Sx Sz g CD step
global N w_eta w_s
% Z = [x'; z'; vx'; vz'; u1'; u2'; u3'; sigma']; % 8*N
X     = Z(1:4,:); % 7*N
x     = Z(1,:)'; % N*1
z     = Z(2,:)'; % N*1
vx    = Z(3,:)'; % N*1
vz    = Z(4,:)'; % N*1
u1    = Z(5,:)'; % N*1
u2    = Z(6,:)'; % N*1
u3    = Z(7,:)'; % N*1
sigma = Z(8,1)'; % 1*1
U   = Z(5:7,:); % N*3

J=0;

Phi=0;
for i = 1:N-1
    x01  = X(1,i);
    z01  = X(2,i);
    vx01 = X(3,i);
    vz01 = X(4,i);

    x02  = X(1,i+1);
    z02  = X(2,i+1);
    vx02 = X(3,i+1);
    vz02 = X(4,i+1);
        
        
    u1_01 = U(1,i);
    u2_01 = U(2,i);
    u3_01 = U(3,i);
        
    u1_02 = U(1,i+1);
    u2_02 = U(2,i+1);
    u3_02 = U(3,i+1);
    

    f1 = [vx01; ...
          vz01; ...
          -rho*vx01^2*CD*Sx/(2*m); ...
          -rho*vz01^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_01);(1/m*u2_01)];
    B1 = zeros(4,3);
    B1(3,1) = 1/m;
    B1(4,2) = 1/m;
    
    f2 =[vx02; ...
         vz02; ...
         -rho*vx02^2*CD*Sx/(2*m); ...
         -rho*vz02^2*CD*Sz/(2*m)-g] + [0;0;(1/m*u1_02);(1/m*u2_02)];
    
    
    
    f = X(:,i+1) - X(:,i) - 0.5*step*(sigma*f1+sigma*f2); % 4*1
    J = J + 1/1e9*step*1/2*U(3,i+1)^2*sigma;
    
    Phi = Phi + mu*norm(f,1) +J;
    
end

end