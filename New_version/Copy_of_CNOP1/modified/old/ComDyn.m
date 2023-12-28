function [H1, H2, G1, G2, b1, b2] = ComDyn(x01,z01,vx01, vz01, x02, z02, vx02, vz02, u1_01, u2_01, u1_02, u2_02, sigma, Sigma, step, m, rho, CD, Sx, Sz, g)
        
                % A_i, B_i, b_i
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
        


end