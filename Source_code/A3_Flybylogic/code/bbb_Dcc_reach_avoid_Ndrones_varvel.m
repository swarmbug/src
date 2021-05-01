%% start reach avoid example for a multiquads with eth tracker++


%clc;
close all;clear all;
N_drones = 4;
%addpath('../casadi-linux-matlabR2014b-v3.5.5')
import casadi.*
addpath('../MiscFunctions');
addpath('../Maps_mrsl');
addpath(genpath('../SmoothRobustness'));
%%
disp('Initializing...');
map_name = 'Maps_mrsl/map_test_2.txt';
% map_name = 'Maps_mrsl/map_test.txt';
H_formula = 6; %H seconds %works with 5 for d=2, but use small C and then recompute. 6 is ok
obs = getObstacles(map_name);
map = load_map(map_name, .5, .5, 0);
close all;
% plot_path(map, []);
close all;
goal.stop = [1.75;1.75;0.75]; %map end point
goal.ds = .25; %thickeness
goal.lb = goal.stop-goal.ds;goal.ub = goal.stop+goal.ds;
goal.poly = Polyhedron('lb',goal.lb,'ub',goal.ub);
% hold on;
% plot(goal.poly,'Color','green','alpha',0.5);
% hold on;

% for i = 1:size(obs,1)
% plot(obs{i}.shape,'Color','red','alpha',0.5);
% hold on;
% end


data = map.boundary;
marg = 0.25; % 0.25 -> (CJ) this is the safety distance
% axis ([data(1)-marg data(4)+marg data(2)-marg data(5)+marg data(3)-marg
% data(6)+marg]); %(CJ) debug
clear data;
h = 1/20; %dt


%% traj gen constraints in casadi
disp('Formulating in Casadi...');

%initialize tracker variables
T = 1; %1s duration of motion
M1 = (1/2*T^5)*[90 0 -15*T^2;-90*T 0 15*T^3;30*T^2 0 -3*T^4];

%tracker limits
max_per_axis = 1; % (CJ) param
max_vel = .751; % .751 -> 3 * .751(CJ) param
optParams.max_vel = max_vel;
max_accl = 1; % (CJ) param
optParams.max_accl = max_accl;
% from vel constraints on pf
K1_T = (90/48)*(1/T) - (90/12)*(1/T) +(30/4)*(1/T);
optParams.K1_T = K1_T;
% from accl constraints on pf
aa = (90/4)*(1/T^5);
bb = -(90/2)*(1/T^4);
cc = (30/2)*(1/T^3);
tp1 = (-bb+sqrt(bb^2-4*aa*cc))/(2*aa);
tp2 = (-bb-sqrt(bb^2-4*aa*cc))/(2*aa);
t_prime = tp1*(tp1>=0)*(tp1<=T) + tp2*(tp2>=0)*(tp2<=T); %pick the right one
K2_tprime = (90/12)*(t_prime^3)/(T^5) - (90/4)*(t_prime^2)/(T^4) + ...
    (30/2)*(t_prime^1)/(T^3);
optParams.K2_tprime = K2_tprime;
%p0 = [-1.5;0;1]; %init position
p0 = zeros(3,N_drones);
p0(:,1) = [-1.20;-1.25;0.8];%[-1.20;-1.25;1.75]; %init position
p0(:,2) = [1.25;-1.25;0.8];%[1.25;-1.25;1.75];
p0(:,3) = [0;1.25;1.75];%[1.25;1.25;1.75];
p0(:,4) = [0;1.25;1.00];%[1.25;1.25;1.75];
p0(:,5) = [1.25;0;1.75];
p0(:,6) = [0;-1.25;1.75];
p0(:,7) = [-0.5;1.25;1.5];
p0(:,8) = [1.75;1.75;1.75];
p0(:,9) = [1;1;1.75];
p0(:,10)= [0;0;1.75];
p0(:,11) = [-1.25;-1.0;1.75];
p0(:,12) = [1;-1;1.75];
if(0) %rand init of quads
   p0 = random_init_generator(map,obs,N_drones); 
    
end
disp("p0");
disp(p0(:,1));
disp("p1-x");
disp(p0(1:1,1)); %the very last number indicates the agent
disp(p0(1,1:N_drones)); 

% hold on;
% plot3(p0(1, 1:N_drones), p0(2, 1:N_drones), p0(3, 1:N_drones), 'b*');
%init vels (zero)
v0 = zeros(3,N_drones);


%(1/(T^5))*[720 -360*T 60*T^2;-360*T 168*T^2 -24*T^3;...
    %60*T^2 -24*T^3 3*T^4];
da = 0;
dv = 0; %start and stop from/at rest

Nsteps = H_formula*(T/h);
Nsteps = round(Nsteps);
% Populate optParams structure % (CJ) param
optParams.T = T;
optParams.M1 = M1;
optParams.da = da;
optParams.dv = dv;
optParams.H_formula = H_formula;
optParams.N_per_T = T/h;
optParams.goal = goal;
optParams.obs = obs;
optParams.map = map;
optParams.max_vel = max_vel;
optParams.max_per_axis = max_per_axis;
optParams.obs_lb_N = repmat(obs{1}.lb,Nsteps+1,1);
optParams.obs_ub_N = repmat(obs{1}.ub,Nsteps+1,1);
optParams.obs = Polyhedron('lb',obs{1}.lb,'ub',obs{1}.ub);
optParams.goal.goal_lb_N = repmat(goal.lb',Nsteps+1,1);
optParams.goal.goal_ub_N = repmat(goal.ub',Nsteps+1,1);
optParams.N_drones = N_drones;
optParams.d_min = 0.2; %min mutual separation % (CJ) param
p_0 = MX.sym('p_0',3,1);
v_0 = MX.sym('v_0',3,1);

w = [];
v = [];
lbw = [];
ubw = [];
lbv = [];
ubv = [];
g = [];
lbg = [];
ubg = [];
Clen = 3*(H_formula+1);
optParams.Clen = Clen;
for d = 1:N_drones
    lbw = [lbw;p0(:,d)];
    ubw = [ubw;p0(:,d)];
    lbv = [lbv;v0(:,d)];
    ubv = [ubv;v0(:,d)];
    
    p_0 = MX.sym(['p_' num2str(d) '_' num2str(0)],3,1);
    v_0 = MX.sym(['v_' num2str(d) '_' num2str(0)],3,1);
    
    w = [w;p_0];
    v = [v;v_0];
    
    for k = 1:H_formula
        p_k = MX.sym(['p_' num2str(d) '_' num2str(k)],3,1);
        v_k = MX.sym(['v_' num2str(d) '_' num2str(k)],3,1);
        
        w = [w;p_k];
        v = [v;v_k];
        % dp for all axes
        dp_x = w(k*3+1+(d-1)*Clen) - w((k-1)*3+1+(d-1)*Clen);
        dp_y = w(k*3+2+(d-1)*Clen) - w((k-1)*3+2+(d-1)*Clen);
        dp_z = w(k*3+3+(d-1)*Clen) - w((k-1)*3+3+(d-1)*Clen);
        
        px_k = w(k*3+1+(d-1)*Clen);px_km1 = w((k-1)*3+1+(d-1)*Clen);
        py_k = w(k*3+2+(d-1)*Clen);py_km1 = w((k-1)*3+2+(d-1)*Clen);
        pz_k = w(k*3+3+(d-1)*Clen);pz_km1 = w((k-1)*3+3+(d-1)*Clen);
        % dv for all axes
        
        dv_x = v(k*3+1+(d-1)*Clen) - v((k-1)*3+1+(d-1)*Clen);
        dv_y = v(k*3+2+(d-1)*Clen) - v((k-1)*3+2+(d-1)*Clen);
        dv_z = v(k*3+3+(d-1)*Clen) - v((k-1)*3+3+(d-1)*Clen);
        
        vx_k = v(k*3+1+(d-1)*Clen);vx_km1 = v((k-1)*3+1+(d-1)*Clen);
        vy_k = v(k*3+2+(d-1)*Clen);vy_km1 = v((k-1)*3+2+(d-1)*Clen);
        vz_k = v(k*3+3+(d-1)*Clen);vz_km1 = v((k-1)*3+3+(d-1)*Clen);
        
        % constants for all 3 axes
        al_x = M1(1,:)*[dp_x-T*vx_km1;0;da];
        be_x = M1(2,:)*[dp_x-T*vx_km1;0;da];
        gam_x = M1(3,:)*[dp_x-T*vx_km1;0;da];
        al_y = M1(1,:)*[dp_y-T*vy_km1;0;da];
        be_y = M1(2,:)*[dp_y-T*vy_km1;0;da];
        gam_y = M1(3,:)*[dp_y-T*vy_km1;0;da];
        al_z = M1(1,:)*[dp_z-T*vz_km1;0;da];
        be_z = M1(2,:)*[dp_z-T*vz_km1;0;da];
        gam_z = M1(3,:)*[dp_z-T*vz_km1;0;da];
        
        %v_fs
        vf_x = (al_x/24)*T^4 + (be_x/6)*T^3 + (gam_x/2)*T^2 + vx_km1;
        vf_y = (al_y/24)*T^4 + (be_y/6)*T^3 + (gam_y/2)*T^2 + vy_km1;
        vf_z = (al_z/24)*T^4 + (be_z/6)*T^3 + (gam_z/2)*T^2 + vz_km1;
        % Constraints per axis and velocity dynamics
        
        g = [g;... % append
             K1_T*dp_x+(1-T*K1_T)*vx_km1;... %from vel constraints on dp
             K1_T*dp_y+(1-T*K1_T)*vy_km1;... 
             K1_T*dp_z+(1-T*K1_T)*vz_km1;... 
             K2_tprime*dp_x-T*K2_tprime*vx_km1;... %from acc constraints
             K2_tprime*dp_y-T*K2_tprime*vy_km1;...
             K2_tprime*dp_z-T*K2_tprime*vz_km1;...
             [vx_k-vf_x;vy_k-vf_y;vz_k-vf_z]]; %from vel dynamics
        lbg = [lbg;-max_vel*ones(3,1);-max_accl*ones(3,1);zeros(3,1)];
        ubg = [ubg;+max_vel*ones(3,1);+max_accl*ones(3,1);zeros(3,1)];
        
         
        
        % overall bounds on movement
        lbw = [lbw;map.boundary(1:3)'+[0;0;.45]];
        ubw = [ubw;map.boundary(4:6)'];
        if(k<H_formula)
        lbv = [lbv;-ones(3,1)*max_vel];
        ubv = [ubv;+ones(3,1)*max_vel];
        else
           lbv = [lbv;zeros(3,1)]; 
           ubv = [ubv;zeros(3,1)];
        end
    end
end

%temp constraints to ensure end point, removed for the real stuff
% lbw(end-3+1:end) = goal.stop;
% ubw(end-3+1:end) = goal.stop;

var = [w;v];
var_ub = [ubw;ubv];
var_lb = [lbw;lbv];
%% Casadi stuff
opts.ipopt.print_level = 5;
opts.print_time = false;
opts.expand = false;
options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 2000, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
    'print_level',0)); %mumps, limited-memory
options.print_time = false;
prob = struct('f', cost_reach_avoid_Ndrones_varvel(var,optParams), 'x', var, 'g', g);
solver = nlpsol('solver', 'ipopt', prob,options);

%% 
disp('Getting init solution...');
w0 = [];
vv0 = [];
for i = 1:N_drones
    [temp tempv] = get_init_waypoints_nonzerovel([p0(:,i);v0(:,i)],optParams); % (CJ) this is the first line
    if(sum(isnan(temp))>0 || sum(isnan(tempv))>0)
    disp('init infeasible');
    keyboard;
    end
    w0 = [w0;temp];
    vv0 = [vv0;tempv];
    pos0{i} = reshape(temp,3,H_formula+1);
    vel0{i} = reshape(tempv,3,H_formula+1);
%     hold all;
%     plot3(pos0{i}(1,:),pos0{i}(2,:),pos0{i}(3,:),'-.');
end

var0 = [w0;vv0];

%enforce stop
var_lb(end-2:end) = zeros(3,1);
var_ub(end-2:end) = zeros(3,1);
%% Solve the NLP
disp('Solving...');
tic;
sol = solver('x0',var0,'lbx', var_lb, 'ubx', var_ub,...
    'lbg', lbg, 'ubg', ubg);

time_taken = toc;
w_opt = full(sol.x);
%%
disp('Plotting...');
if(1)
%     pause;
   
    waypoints = cell(optParams.N_drones,1);
    mar{1} = 'k*';
    mar{2} = 'g*';
    mar{3} = 'r*';
    mar{4} = 'b*';
    mar{5} = 'c*';
    mar{6} = 'm*';
    mar{7} = 'y*';
    mar{8} = 'r*';
    mar{9} = 'g*';
    mar{10} = 'k*';
    mar{11} = 'b*';
    mar{12} = 'c*';
    mar{13} = 'k*';
    mar{14} = 'g*';
    mar{15} = 'r*';
    mar{16} = 'b*';
    for d = 1:N_drones
        
        waypoints{d} = reshape(w_opt(1+(d-1)*(H_formula+1)*3:...
            (d)*(H_formula+1)*3),3,H_formula+1);
%         hold all;
%         plot3(waypoints{d}(1,:),waypoints{d}(2,:),waypoints{d}(3,:),mar{d});
        
        
    end
    [negative_rob,xx,yy,zz] = cost_reach_avoid_Ndrones_varvel(w_opt,optParams);
    
end

dlmwrite('original_coor.csv','This is xx' ,'delimiter',',','-append'); 
dlmwrite('original_coor.csv',xx','delimiter',',','-append'); 
dlmwrite('original_coor.csv','This is yy' ,'delimiter',',','-append'); 
dlmwrite('original_coor.csv',yy','delimiter',',','-append'); 
dlmwrite('original_coor.csv','This is zz' ,'delimiter',',','-append'); 
dlmwrite('original_coor.csv',zz','delimiter',',','-append'); 
 
%%%%%%%%%%%%%% input: agent들 의최초좌표 들16개, 세트. + 요망하는,시점. 20 -> 120 * 20 / 100
%%%%%%%%%%%%% A.csv
for current_t = 1:100
    %current_t = 20;
disp(current_t);
desired_time = current_t + 1;

%desired_time = 120 * 20 / (120 - 20); %
disp("xx(current_t ,:)");
disp(xx(current_t ,:));
current_xx = xx(current_t ,:);
current_yy = yy(current_t ,:);
current_zz = zz(current_t ,:);
% disp([current_xx(:,1);current_yy(:,1);current_zz(:,1)]);

does_obstacle_exist = 1;

[xx_1, yy_1, zz_1] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, current_xx, current_yy, current_zz, does_obstacle_exist);

if(0)
    dlmwrite('A.csv','desired_time' ,'delimiter',',','-append'); 
    dlmwrite('A.csv',desired_time,'delimiter',',','-append'); 
    dlmwrite('A.csv','This is xx' ,'delimiter',',','-append'); 
    dlmwrite('A.csv',xx_1','delimiter',',','-append'); 
    dlmwrite('A.csv','This is yy' ,'delimiter',',','-append'); 
    dlmwrite('A.csv',yy_1','delimiter',',','-append'); 
    dlmwrite('A.csv','This is zz' ,'delimiter',',','-append'); 
    dlmwrite('A.csv',zz_1','delimiter',',','-append');
end


%%%%%%%%%%%%% B.csv

modified_current_xx_2 = xx(current_t,:);
modified_current_yy_2 = yy(current_t,:);
modified_current_zz_2 = zz(current_t,:);
modified_current_xx_2(2) = 0; %make 2nd agent's coord. 0, 0, 0
modified_current_yy_2(2) = 0;
modified_current_zz_2(2) = 0;

modified_current_xx_3 = xx(current_t,:);
modified_current_yy_3 = yy(current_t,:);
modified_current_zz_3 = zz(current_t,:);
modified_current_xx_3(3) = 0;
modified_current_yy_3(3) = 0;
modified_current_zz_3(3) = 0;

modified_current_xx_4 = xx(current_t,:);
modified_current_yy_4 = yy(current_t,:);
modified_current_zz_4 = zz(current_t,:);
modified_current_xx_4(4) = 0;
modified_current_yy_4(4) = 0;
modified_current_zz_4(4) = 0;

modified_current_xx_5 = xx(current_t,:);
modified_current_yy_5 = yy(current_t,:);
modified_current_zz_5 = zz(current_t,:);
modified_current_xx_5(5) = 0;
modified_current_yy_5(5) = 0;
modified_current_zz_5(5) = 0;
% 
% modified_current_xx_6 = xx(current_t,:);
% modified_current_yy_6 = yy(current_t,:);
% modified_current_zz_6 = zz(current_t,:);
% modified_current_xx_6(6) = 0;
% modified_current_yy_6(6) = 0;
% modified_current_zz_6(6) = 0;
% 
% modified_current_xx_7 = xx(current_t,:);
% modified_current_yy_7 = yy(current_t,:);
% modified_current_zz_7 = zz(current_t,:);
% modified_current_xx_7(7) = 0;
% modified_current_yy_7(7) = 0;
% modified_current_zz_7(7) = 0;
% 
% modified_current_xx_8 = xx(current_t,:);
% modified_current_yy_8 = yy(current_t,:);
% modified_current_zz_8 = zz(current_t,:);
% modified_current_xx_8(8) = 0;
% modified_current_yy_8(8) = 0;
% modified_current_zz_8(8) = 0;
% 
% modified_current_xx_9 = xx(current_t,:);
% modified_current_yy_9 = yy(current_t,:);
% modified_current_zz_9 = zz(current_t,:);
% modified_current_xx_9(9) = 0;
% modified_current_yy_9(9) = 0;
% modified_current_zz_9(9) = 0;
% 
% modified_current_xx_10 = xx(current_t,:);
% modified_current_yy_10 = yy(current_t,:);
% modified_current_zz_10 = zz(current_t,:);
% modified_current_xx_10(10) = 0;
% modified_current_yy_10(10) = 0;
% modified_current_zz_10(10) = 0;
% 
% modified_current_xx_11 = xx(current_t,:);
% modified_current_yy_11 = yy(current_t,:);
% modified_current_zz_11 = zz(current_t,:);
% modified_current_xx_11(11) = 0;
% modified_current_yy_11(11) = 0;
% modified_current_zz_11(11) = 0;
% 
% modified_current_xx_12 = xx(current_t,:);
% modified_current_yy_12 = yy(current_t,:);
% modified_current_zz_12 = zz(current_t,:);
% modified_current_xx_12(12) = 0;
% modified_current_yy_12(12) = 0;
% modified_current_zz_12(12) = 0;
% 
% modified_current_xx_13 = xx(current_t,:);
% modified_current_yy_13 = yy(current_t,:);
% modified_current_zz_13 = zz(current_t,:);
% modified_current_xx_13(13) = 0;
% modified_current_yy_13(13) = 0;
% modified_current_zz_13(13) = 0;
% 
% modified_current_xx_14 = xx(current_t,:);
% modified_current_yy_14 = yy(current_t,:);
% modified_current_zz_14 = zz(current_t,:);
% modified_current_xx_14(14) = 0;
% modified_current_yy_14(14) = 0;
% modified_current_zz_14(14) = 0;
% 
% modified_current_xx_15 = xx(current_t,:);
% modified_current_yy_15 = yy(current_t,:);
% modified_current_zz_15 = zz(current_t,:);
% modified_current_xx_15(15) = 0;
% modified_current_yy_15(15) = 0;
% modified_current_zz_15(15) = 0;
% 
% modified_current_xx_16 = xx(current_t,:);
% modified_current_yy_16 = yy(current_t,:);
% modified_current_zz_16 = zz(current_t,:);
% modified_current_xx_16(16) = 0;
% modified_current_yy_16(16) = 0;
% modified_current_zz_16(16) = 0;

% 0으로다바꿨구나, 그래서_a2의좌표를0으로_바꾼_애들을_넣어준상황에서의_a1의_값이구나.
% disp("modified_current_xx_2");
% disp(modified_current_xx_2);
other_agents = 1;
if(other_agents)
    [xx_2, yy_2, zz_2] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_2, modified_current_yy_2, modified_current_zz_2, does_obstacle_exist);
    [xx_3, yy_3, zz_3] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_3, modified_current_yy_3, modified_current_zz_3, does_obstacle_exist);
    [xx_4, yy_4, zz_4] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_4, modified_current_yy_4, modified_current_zz_4, does_obstacle_exist);
    [xx_5, yy_5, zz_5] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_5, modified_current_yy_5, modified_current_zz_5, does_obstacle_exist);
end

if(0)
    dlmwrite('B.csv','desired_time' ,'delimiter',',','-append'); 
    dlmwrite('B.csv',desired_time,'delimiter',',','-append'); 
    dlmwrite('B.csv','This is xx' ,'delimiter',',','-append'); 
    dlmwrite('B.csv',xx_2','delimiter',',','-append'); 
    dlmwrite('B.csv','This is yy' ,'delimiter',',','-append'); 
    dlmwrite('B.csv',yy_2','delimiter',',','-append'); 
    dlmwrite('B.csv','This is zz' ,'delimiter',',','-append'); 
    dlmwrite('B.csv',zz_2','delimiter',',','-append'); 
end


% [xx_6, yy_6, zz_6] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_6, modified_current_yy_6, modified_current_zz_6, does_obstacle_exist);
% [xx_7, yy_7, zz_7] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_7, modified_current_yy_7, modified_current_zz_7, does_obstacle_exist);
% [xx_8, yy_8, zz_8] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_8, modified_current_yy_8, modified_current_zz_8, does_obstacle_exist);
% [xx_9, yy_9, zz_9] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_9, modified_current_yy_9, modified_current_zz_9, does_obstacle_exist);
% [xx_10, yy_10, zz_10] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_10, modified_current_yy_10, modified_current_zz_10, does_obstacle_exist);
% [xx_11, yy_11, zz_11] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_11, modified_current_yy_11, modified_current_zz_11, does_obstacle_exist);
% [xx_12, yy_12, zz_12] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_12, modified_current_yy_12, modified_current_zz_12, does_obstacle_exist);
% [xx_13, yy_13, zz_13] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_13, modified_current_yy_13, modified_current_zz_13, does_obstacle_exist);
% [xx_14, yy_14, zz_14] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_14, modified_current_yy_14, modified_current_zz_14, does_obstacle_exist);
% [xx_15, yy_15, zz_15] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_15, modified_current_yy_15, modified_current_zz_15, does_obstacle_exist);
% [xx_16, yy_16, zz_16] = aaa_fake_testbed_Copy_of_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_16, modified_current_yy_16, modified_current_zz_16, does_obstacle_exist);

if(other_agents)

delta_about_a2 = sqrt( (xx_1(desired_time,1) - xx_2(desired_time,1))^2 + (yy_1(desired_time,1) - yy_2(desired_time,1))^2 + (zz_1(desired_time,1) - zz_2(desired_time,1))^2 );
delta_about_a3 = sqrt( (xx_1(desired_time,1) - xx_3(desired_time,1))^2 + (yy_1(desired_time,1) - yy_3(desired_time,1))^2 + (zz_1(desired_time,1) - zz_3(desired_time,1))^2 );
delta_about_a4 = sqrt( (xx_1(desired_time,1) - xx_4(desired_time,1))^2 + (yy_1(desired_time,1) - yy_4(desired_time,1))^2 + (zz_1(desired_time,1) - zz_4(desired_time,1))^2 );
delta_about_a5 = sqrt( (xx_1(desired_time,1) - xx_5(desired_time,1))^2 + (yy_1(desired_time,1) - yy_5(desired_time,1))^2 + (zz_1(desired_time,1) - zz_5(desired_time,1))^2 );
end
% disp("xx_1(desired_time,1)");
% disp(xx_1(desired_time,1));
% disp("xx_2(desired_time,1)");
% disp(xx_2(desired_time,1));
% delta_about_a6 = sqrt( (xx_1(desired_time,1) - xx_6(desired_time,1))^2 + (yy_1(desired_time,1) - yy_6(desired_time,1))^2 + (zz_1(desired_time,1) - zz_6(desired_time,1))^2 );
% delta_about_a7 = sqrt( (xx_1(desired_time,1) - xx_7(desired_time,1))^2 + (yy_1(desired_time,1) - yy_7(desired_time,1))^2 + (zz_1(desired_time,1) - zz_7(desired_time,1))^2 );
% delta_about_a8 = sqrt( (xx_1(desired_time,1) - xx_8(desired_time,1))^2 + (yy_1(desired_time,1) - yy_8(desired_time,1))^2 + (zz_1(desired_time,1) - zz_8(desired_time,1))^2 );
% delta_about_a9 = sqrt( (xx_1(desired_time,1) - xx_9(desired_time,1))^2 + (yy_1(desired_time,1) - yy_9(desired_time,1))^2 + (zz_1(desired_time,1) - zz_9(desired_time,1))^2 );
% delta_about_a10 = sqrt( (xx_1(desired_time,1) - xx_10(desired_time,1))^2 + (yy_1(desired_time,1) - yy_10(desired_time,1))^2 + (zz_1(desired_time,1) - zz_10(desired_time,1))^2 );
% delta_about_a11 = sqrt( (xx_1(desired_time,1) - xx_11(desired_time,1))^2 + (yy_1(desired_time,1) - yy_11(desired_time,1))^2 + (zz_1(desired_time,1) - zz_11(desired_time,1))^2 );
% delta_about_a12 = sqrt( (xx_1(desired_time,1) - xx_12(desired_time,1))^2 + (yy_1(desired_time,1) - yy_12(desired_time,1))^2 + (zz_1(desired_time,1) - zz_12(desired_time,1))^2 );
% delta_about_a13 = sqrt( (xx_1(desired_time,1) - xx_13(desired_time,1))^2 + (yy_1(desired_time,1) - yy_13(desired_time,1))^2 + (zz_1(desired_time,1) - zz_13(desired_time,1))^2 );
% delta_about_a14 = sqrt( (xx_1(desired_time,1) - xx_14(desired_time,1))^2 + (yy_1(desired_time,1) - yy_14(desired_time,1))^2 + (zz_1(desired_time,1) - zz_14(desired_time,1))^2 );
% delta_about_a15 = sqrt( (xx_1(desired_time,1) - xx_15(desired_time,1))^2 + (yy_1(desired_time,1) - yy_15(desired_time,1))^2 + (zz_1(desired_time,1) - zz_15(desired_time,1))^2 );
% delta_about_a16 = sqrt( (xx_1(desired_time,1) - xx_16(desired_time,1))^2 + (yy_1(desired_time,1) - yy_16(desired_time,1))^2 + (zz_1(desired_time,1) - zz_16(desired_time,1))^2 );



[xx_o, yy_o, zz_o] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, current_xx, current_yy, current_zz, 0);
delta_about_o = sqrt( (xx_1(desired_time,1) - xx_o(desired_time,1))^2 + (yy_1(desired_time,1) - yy_o(desired_time,1))^2 + (zz_1(desired_time,1) - zz_o(desired_time,1))^2 );


temp_sum = delta_about_a2 + delta_about_a3 + delta_about_a4 + delta_about_a5 + delta_about_o;

temp_dcc_2 = delta_about_a2 / temp_sum;
temp_dcc_3 = delta_about_a3 / temp_sum;
temp_dcc_4 = delta_about_a4 / temp_sum;
temp_dcc_5 = delta_about_a5 / temp_sum;
temp_dcc_o = delta_about_o / temp_sum;

dlmwrite('delta.csv','This is delta' ,'delimiter',',','-append'); 
if(other_agents)
dlmwrite('delta.csv',delta_about_a2,'delimiter',',','-append'); 
dlmwrite('delta.csv',delta_about_a3,'delimiter',',','-append'); 
dlmwrite('delta.csv',delta_about_a4,'delimiter',',','-append'); 
dlmwrite('delta.csv',delta_about_a5,'delimiter',',','-append'); 
end
% dlmwrite('delta.csv',delta_about_a6,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a7,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a8,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a9,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a10,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a11,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a12,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a13,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a14,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a15,'delimiter',',','-append'); 
% dlmwrite('delta.csv',delta_about_a16,'delimiter',',','-append'); 

dlmwrite('delta.csv',delta_about_o,'delimiter',',','-append');

dlmwrite('dcc.csv',[temp_dcc_2, temp_dcc_3, temp_dcc_4, temp_dcc_5, temp_dcc_o],'delimiter',',','-append');
end