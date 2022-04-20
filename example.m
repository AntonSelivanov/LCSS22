% This MATLAB program checks the feasibility of LMIs from Theorem 1 of the paper 
% A. Selivanov and E. Fridman, "PDE-based deployment of multi-agents measuring relative position to one neighbour," IEEE Control Systems Letters, 2022. 
%% System parameters 
nu=.1;                                  % propagation speed from (9)
L=.2;                                   % the Lipschitz constant (see assumptions below (2))
k=1;                                    % controller gain from (3) 
tauM=.89;                               % upper bound on the delay in (17) 
alpha=.1;                               % decay rate 
%% Check LMIs 
if LMI_th1(nu,L,k,tauM,alpha)
    disp('LMIs are feasible')
else
    disp('LMIs are not feasible')
end