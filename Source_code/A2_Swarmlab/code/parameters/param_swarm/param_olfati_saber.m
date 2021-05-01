%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Olfati Saber parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Coeff for velocity matching
p_swarm.c_vm  = 6; %(CJ) 3 -> 6, 0.6


p_swarm.a     = 1; %(CJ) 1 -> 2, 0.2
p_swarm.b     = 5; %(CJ) 5 -> 10, 2.5

p_swarm.c     = (p_swarm.b-p_swarm.a)/(2*sqrt(p_swarm.a*p_swarm.b));
p_swarm.delta = 0.2; %(CJ) 0.2 -> 4, 0.04
p_swarm.k     = 2; %(CJ) 2 -> 4, 0.4

% Velocity of migration - replace the velocity matching and it uses the
% same gain P.c_cm
% P.v_migration = [0 4 0]';  

% Obstacles parameters (CJ) It looks like things related to how the drone
% consider about obstacles...!! Also, it has little difference from another
% algorithm: vicsek.
p_swarm.r0 = 10; %(CJ) 10 -> 20, 2 ::: 25 is proper fix!

p_swarm.lambda = 1; %(CJ) 1 -> 0.2              % (0,1]

p_swarm.c_pm_obs = 5; %(CJ) 5 -> 10, 1

p_swarm.c_vm_obs = p_swarm.c_vm;
