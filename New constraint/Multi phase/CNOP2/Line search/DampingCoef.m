function c = DampingCoef(Zk1, Zk2, mu, obj)
global cf lambda

c = 1;
phi_old = MeritFunc(Zk1, mu, obj);
P       = Zk2 - Zk1; % 8*N get search direction

%% Backtracking Line Search (Wolfe Conditions)
% (mu = 10, cf = 0.5, lambda = 0.4) 
eps = 1e-6;
Zk_eps = Zk1 + eps*P;
phi_eps = MeritFunc(Zk_eps, mu, obj);
D_phi = (phi_eps-phi_old)/eps;
for i = 1:10
    Zk = Zk1 + c*P;
    phi_new = MeritFunc(Zk, mu, obj);
    if phi_new <= phi_old + lambda*c*D_phi
        break;
    else
        c = cf*c;
    end
end


% %% Backtracking Line Search (Strong Wolfe Conditions)
% % (mu = 10, cf = 0.5, lambda = 0.4, lambda2 = 0.8) works good, most of c is equal to 1
% lambda2 = 0.8; % 0 < lambda < lambda2 < 1
% eps = 1e-6;
% Zk_eps = Zk1 + eps*P;
% phi_eps = MeritFunc(Zk_eps, mu);
% D_phi1 = (phi_eps-phi_old)/eps;
% for i = 1:10
%     Zk = Zk1 + c*P;
%     phi_new = MeritFunc(Zk, mu);
%     Zk_eps2 = Zk + eps*P;
%     phi_eps2 = MeritFunc(Zk_eps2, mu);
%     D_phi2 = (phi_eps2-phi_new)/eps;
%     if phi_new <= phi_old + lambda*c*D_phi1 && abs(D_phi2) <= lambda2*abs(D_phi1)
%         break;
%     else
%         c = cf*c;
%     end
% end


%% Backtracking Line Search (Goldstein Conditions, 0 < eta < 0.5)
% (mu = 10, cf = 0.5, lambda = 0.4 or 0.2) 
% eps = 1e-6;
% Zk_eps = Zk1 + eps*P;
% phi_eps = MeritFunc(Zk_eps, mu);
% D_phi = (phi_eps-phi_old)/eps;
% for i = 1:10
%     Zk = Zk1 + c*P;
%     phi_new = MeritFunc(Zk, mu);
%     if phi_old + (1-lambda)*c*D_phi <= phi_new <= phi_old + lambda*c*D_phi
%         break;
%     else
%         c = cf*c;
%     end
% end


end