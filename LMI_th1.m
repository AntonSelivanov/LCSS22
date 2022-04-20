function res=LMI_th1(nu,L,k,tauM,alpha)
% This MATLAB program checks the feasibility of LMIs from Theorem 1 of the paper 
% A. Selivanov and E. Fridman, "PDE-based deployment of multi-agents measuring relative position to one neighbour," IEEE Control Systems Letters, 2022. 

% The program uses YALMIP parser (http://users.isy.liu.se/johanl/yalmip/)
% and SeDuMi solver (http://sedumi.ie.lehigh.edu/)

% Input: 
% nu        - propagation speed from (9)
% L         - the Lipschitz constant (see assumptions below (2))
% k         - controller gain from (3) 
% tauM      - upper bound on the delay in (17) 
% alpha     - decay rate 

% Output: 
% res =1 (true) if feasible, =0 (false) otherwise
%% The LMI
sdpvar p r eta

Phi=blkvar; 
Phi(1,1)=nu*p+eta*L^2+2*alpha-r*exp(-2*alpha*tauM); 
Phi(1,2)=-k+r*exp(-2*alpha*tauM); 
Phi(1,3)=1; 
Phi(2,2)=tauM^2*r*k^2-r*exp(-2*alpha*tauM); 
Phi(2,3)=-tauM^2*r*k; 
Phi(3,3)=tauM^2*r-eta;
Phi=sdpvar(Phi); 
%% Solution of LMIs
LMIs=[p>=0, r>=0, Phi<=0]; 

options=sdpsettings('solver','sedumi','verbose',0); 
sol=solvesdp(LMIs,[],options); 

res=false; 
if sol.problem == 0
    [primal,dual]=check(LMIs); 
    res = min(primal)>0 && min(dual)>0; 
end