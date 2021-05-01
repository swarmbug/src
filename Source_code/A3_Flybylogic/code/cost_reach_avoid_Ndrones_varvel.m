function [negative_rob,xx,yy,zz] = cost_reach_avoid_Ndrones_varvel(var,optParams)
import casadi.*
type_of = isfloat(var); %0 for casadi
w = var(1:numel(var)/2);
v = var(numel(var)/2+1:end);
N_per_T = round(optParams.N_per_T);
C = 50.0; %const for smooth min/max operation %for 2 drones, use 10 with a period of 5s, casadi is unstable numerically
C1 =50.0; %const for smooth max %20 works for 10 drones, det init points
C2 =50.0; %5 makes no numerical instblty in 12 drones
dT = optParams.T/optParams.N_per_T:optParams.T/optParams.N_per_T:optParams.T;
dv = optParams.dv;
da = optParams.da;
M1 = optParams.M1;
T = optParams.T;
Clen = optParams.Clen;
if(type_of) %if double input
    temp_x = zeros(optParams.N_drones, numel(dT));
    temp_y = zeros(optParams.N_drones, numel(dT));
    temp_z = zeros(optParams.N_drones, numel(dT));
    xx = zeros(numel(dT)+1,optParams.N_drones);
    yy = zeros(numel(dT)+1,optParams.N_drones);
    zz = zeros(numel(dT)+1,optParams.N_drones);
    rho_unsafe = zeros(optParams.N_drones,1);
    rho_goal = zeros(optParams.N_drones,1);
    mutual_distances = zeros(numel(dT)*optParams.H_formula+1,1);
    if(optParams.N_drones>1)
    dists = zeros(nchoosek(optParams.N_drones,2),1);
    end
else
%     temp_x = MX.sym('temp_x',optParams.N_drones, numel(dT));
%     temp_y = MX.sym('temp_y',optParams.N_drones, numel(dT));
%     temp_z = MX.sym('temp_z',optParams.N_drones, numel(dT));
%     xx = MX.sym('xx',numel(dT)*optParams.H_formula+1,optParams.N_drones);
%     yy = MX.sym('yy',numel(dT)*optParams.H_formula+1,optParams.N_drones);
%     zz = MX.sym('zz',numel(dT)*optParams.H_formula+1,optParams.N_drones);
%     rho_unsafe = MX.sym('r_u',optParams.N_drones,1);
%     rho_goal = MX.sym('r_u',optParams.N_drones,1);
%     mutual_distances = MX.sym('msep',numel(dT)*optParams.H_formula+1,1);
%     if(optParams.N_drones>1)
%     dists = MX.sym('mdist',nchoosek(optParams.N_drones,2),1);
%     end
%temp_x = MX.sym('temp_x',optParams.N_drones, numel(dT));
    temp_x = MX.zeros(optParams.N_drones, numel(dT));
    
    %temp_y = MX.sym('temp_y',optParams.N_drones, numel(dT));
    temp_y = MX.zeros(optParams.N_drones, numel(dT));
    
    %temp_z = MX.sym('temp_z',optParams.N_drones, numel(dT));
    temp_z = MX.zeros(optParams.N_drones, numel(dT));
    
%     xx = MX.sym('xx',numel(dT)*optParams.H_formula+1,optParams.N_drones);
xx = MX.zeros(numel(dT)*optParams.H_formula+1,optParams.N_drones);    
% yy = MX.sym('yy',numel(dT)*optParams.H_formula+1,optParams.N_drones);
yy = MX.zeros(numel(dT)*optParams.H_formula+1,optParams.N_drones);    
% zz = MX.sym('zz',numel(dT)*optParams.H_formula+1,optParams.N_drones);
zz = MX.zeros(numel(dT)*optParams.H_formula+1,optParams.N_drones);

    %rho_unsafe = MX.sym('r_u',optParams.N_drones,1);
    rho_unsafe = MX.zeros(optParams.N_drones,1);
    %rho_goal = MX.sym('r_u',optParams.N_drones,1);
    rho_goal = MX.zeros(optParams.N_drones,1);
    %mutual_distances = MX.sym('msep',numel(dT)*optParams.H_formula+1,1);
    mutual_distances = MX.zeros(numel(dT)*optParams.H_formula+1,1);
    if(optParams.N_drones>1)
%     dists = MX.sym('mdist',nchoosek(optParams.N_drones,2),1);
    dists = MX.zeros(nchoosek(optParams.N_drones,2),1);
    end

end

for d = 1:optParams.N_drones
    %init posns
    xx(1,d) = w(1+(d-1)*Clen);
    yy(1,d) = w(2+(d-1)*Clen);
    zz(1,d) = w(3+(d-1)*Clen);
    
    for k = 1:optParams.H_formula
        
        % dp for all axes
        dp_x = w(k*3+1+(d-1)*Clen) - w((k-1)*3+1+(d-1)*Clen) - T*v((k-1)*3+1+(d-1)*Clen);
        dp_y = w(k*3+2+(d-1)*Clen) - w((k-1)*3+2+(d-1)*Clen) - T*v((k-1)*3+2+(d-1)*Clen);
        dp_z = w(k*3+3+(d-1)*Clen) - w((k-1)*3+3+(d-1)*Clen) - T*v((k-1)*3+3+(d-1)*Clen);
        
        % constants for all 3 axes
        al_x = M1(1,:)*[dp_x;dv;da];
        be_x = M1(2,:)*[dp_x;dv;da];
        gam_x = M1(3,:)*[dp_x;dv;da];
        al_y = M1(1,:)*[dp_y;dv;da];
        be_y = M1(2,:)*[dp_y;dv;da];
        gam_y = M1(3,:)*[dp_y;dv;da];
        al_z = M1(1,:)*[dp_z;dv;da];
        be_z = M1(2,:)*[dp_z;dv;da];
        gam_z = M1(3,:)*[dp_z;dv;da];
        
        temp_x(d,:) = (al_x/120)*dT.^5 + (be_x/24)*dT.^4 + ...
            (gam_x/6)*dT.^3   + w((k-1)*3+1+(d-1)*Clen) + ...
            dT*v((k-1)*3+1+(d-1)*Clen); %fix w points
        
        xx(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_x(d,:)'; %20
        
        temp_y(d,:) = (al_y/120)*dT.^5 + (be_y/24)*dT.^4 + ...
            (gam_y/6)*dT.^3   + w((k-1)*3+2+(d-1)*Clen) + ...
            dT*v((k-1)*3+2+(d-1)*Clen); % fix w points
        
        yy(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_y(d,:)';
        
        temp_z(d,:) = (al_z/120)*dT.^5 + (be_z/24)*dT.^4 + ...
            (gam_z/6)*dT.^3   + w((k-1)*3+3+(d-1)*Clen) + ...
            dT*v((k-1)*3+3+(d-1)*Clen); %fix w points
        
        zz(2+(k-1)*N_per_T:k*N_per_T+1,d) = temp_z(d,:)'; 
        
    end
    
    states = [xx(:,d), yy(:,d), zz(:,d)];
    % SPECIFY FORMULA FOR THE OBSTACLE HERE
    % always not unsafe in x y z
    rho_unsafe(d) = alwaysNot(1, states, optParams.obs, 0:numel(dT)*optParams.H_formula, C);
    
    % SPECIFY FORMULA FOR THE GOAL HERE
    % Option 1: eventually goal in x y z
%     I = 0:numel(dT)*optParams.H_formula;
%     rho_goal(d) = eventually(1, states, optParams.goal.poly, I, C); 
    
    % Option 2: always eventually (reach the goal and let other drones to reach it too == leave it)
%     % check that I+J <= H_formula specified in the head file! 
%     Ia = 0; % in seconds
%     Ib = 1; % also seconds
%     Ja = 0; % seconds
%     Jb = 5;
%     I = Ia*numel(dT):Ib*numel(dT); % convertation to time-steps (needed for MTL formulas)
%     J = Ja*numel(dT):Jb*numel(dT);
%     rho_goal(d) = alwaysEventually(states, optParams.goal.poly, I, J, C);
    
    % Option 3: eventually always
    % RESULTS: one can see that final points are not in the goal, but it is
    % still SAT the spec, because some of the drones on second 2 reached
    % the goal and stayed there for one sec, and other reached the goal
    % later (when first drones already left the goal) and stayed there for
    % 1 second. 
    %
    % check that I+J <= H_formula specified in the head file!
    Ia = 0; % in seconds
    Ib = 5; % also seconds
    Ja = 0; % seconds
    Jb = 1;
    I = Ia*numel(dT):Ib*numel(dT); % convertation to time-steps (needed for MTL formulas)
    J = Ja*numel(dT):Jb*numel(dT);
    rho_goal(d) = eventuallyAlways(states, optParams.goal.poly, I, J, C);
end

% pairwise distances
if(optParams.N_drones>1)
combos = nchoosek(1:optParams.N_drones,2);
for p = 1:size(combos,1)
    for k=1:size(xx,1) %for all time steps
    pa = [xx(k,combos(p,1));yy(k,combos(p,1));zz(k,combos(p,1))];
    pb = [xx(k,combos(p,2));yy(k,combos(p,2));zz(k,combos(p,2))];
    mutual_distances(k) = norm(pa-pb,2)-optParams.d_min;
    %   d  
%     dx(:,p) = xx(:,combos(p,1))-xx(:,combos(p,2));
%     dy(:,p) = yy(:,combos(p,1))-yy(:,combos(p,2));
%     dz(:,p) = zz(:,combos(p,1))-zz(:,combos(p,2));
    end
    dists(p) = SmoothMin(mutual_distances,C2);
end
else
   dists = []; 
end

negative_rob = -SmoothMin([rho_unsafe;rho_goal;dists],C);
