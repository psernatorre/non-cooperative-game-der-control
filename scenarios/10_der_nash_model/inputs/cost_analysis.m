clear
clc

%% Load state-space and controllers

load("state_space_and_controllers.mat")

Ts = 0.0001;

scale_factor = (1e6*3600^2); 
% The objective function is in MW^2*seconds. 
% The numbers in Q and R are in dollar/(kWh)^2


%% Get path to save results

mydir  = pwd;
idcs   = strfind(mydir,'/');
newdir = mydir(1:idcs(end)-1);
newdir = newdir + "/outputs/";


%% Analysis 1

% Load simulation results
load("sim_nasheq_model.mat")

% Check the names of the _out variables loaded in the workspace. They
% are used here.

t_intervals =[0.25 1.5;
              1.5 3;
              3 4.5;
              4.5 6.0];

n_intervals = size(t_intervals,1);

results = zeros(n_intervals+1,2+10);

for k = 1:1:n_intervals

    tin = t_intervals(k,1); %Initial time
    tfi = t_intervals(k,2); %Final time

    t   = nm_out.w.Time; %Get the time vector
    tint= (t>=tin & t<tfi); %Get the time interval under analysis

    % States of the DERs
    x_pv1_g     = get_x_g_der(nm_out.x_pv1, tint);
    x_bess2_g   = get_x_g_der(nm_out.x_bess2, tint);
    x_bess3_g   = get_x_g_der(nm_out.x_bess3, tint);
    x_pv4_g     = get_x_g_der(nm_out.x_pv4, tint);
    x_pv5_g     = get_x_g_der(nm_out.x_pv5, tint);
    x_bess6_g   = get_x_g_der(nm_out.x_bess6, tint);
    x_pv7_g     = get_x_g_der(nm_out.x_pv7, tint);
    x_bess8_g   = get_x_g_der(nm_out.x_bess8, tint);
    x_bess9_g   = get_x_g_der(nm_out.x_bess9, tint);
    x_bess10_g  = get_x_g_der(nm_out.x_bess10, tint);

    % Integral of error
    w_g = get_w_g(nm_out.w, tint);

    % Full vector state of the deviation system
    x_g = [x_pv1_g; x_bess2_g; x_bess3_g; x_pv4_g; x_pv5_g; x_bess6_g; x_pv7_g; x_bess8_g; x_bess9_g; x_bess10_g; w_g] ;

    % Control actions
    u_pv1_g     = get_w_g(nm_out.u_pv1, tint);
    u_bess2_g   = get_w_g(nm_out.u_bess2, tint);
    u_bess3_g   = get_w_g(nm_out.u_bess3, tint);
    u_pv4_g     = get_w_g(nm_out.u_pv4, tint);
    u_pv5_g     = get_w_g(nm_out.u_pv5, tint);
    u_bess6_g   = get_w_g(nm_out.u_bess6, tint);
    u_pv7_g     = get_w_g(nm_out.u_pv7, tint);
    u_bess8_g   = get_w_g(nm_out.u_bess8, tint);
    u_bess9_g   = get_w_g(nm_out.u_bess9, tint);
    u_bess10_g  = get_w_g(nm_out.u_bess10, tint);

    % Compute Ji of the deviation system by formula and integration
    PV1.cost    = x_g(:,1)'*PV1.PRic*x_g(:,1);
    BESS2.cost  = x_g(:,1)'*BESS2.PRic*x_g(:,1);
    BESS3.cost  = x_g(:,1)'*BESS3.PRic*x_g(:,1);
    PV4.cost    = x_g(:,1)'*PV4.PRic*x_g(:,1);
    PV5.cost    = x_g(:,1)'*PV5.PRic*x_g(:,1);
    BESS6.cost  = x_g(:,1)'*BESS6.PRic*x_g(:,1);
    PV7.cost    = x_g(:,1)'*PV7.PRic*x_g(:,1);
    BESS8.cost  = x_g(:,1)'*BESS8.PRic*x_g(:,1);
    BESS9.cost  = x_g(:,1)'*BESS9.PRic*x_g(:,1);
    BESS10.cost = x_g(:,1)'*BESS10.PRic*x_g(:,1);

    results(k,1) = tin;
    results(k,2) = tfi;
    results(k,3) = PV1.cost /scale_factor;
    results(k,4) = BESS2.cost/scale_factor;
    results(k,5) = BESS3.cost/scale_factor;
    results(k,6) = PV4.cost/scale_factor;
    results(k,7) = PV5.cost/scale_factor;
    results(k,8) = BESS6.cost/scale_factor;
    results(k,9) = PV7.cost/scale_factor;
    results(k,10) = BESS8.cost/scale_factor;
    results(k,11) = BESS9.cost/scale_factor;
    results(k,12) = BESS10.cost/scale_factor;
end
results(end,1) = t_intervals(1,1);
results(end,2) = t_intervals(end,end);
results(end,3) = sum(results(:,3));
results(end,4) = sum(results(:,4));
results(end,5) = sum(results(:,5));
results(end,6) = sum(results(:,6));
results(end,7) = sum(results(:,7));
results(end,8) = sum(results(:,8));
results(end,9) = sum(results(:,9));
results(end,10) = sum(results(:,10));
results(end,11) = sum(results(:,11));
results(end,12) = sum(results(:,12));

table_results = table;
table_results.start_time = results(:,1);
table_results.end_time = results(:,2);
table_results.PV1  = results(:,3);
table_results.BESS2  = results(:,4);
table_results.BESS3  = results(:,5);
table_results.PV4  = results(:,6);
table_results.PV5  = results(:,7);
table_results.BESS6  = results(:,8);
table_results.PV7  = results(:,9);
table_results.BESS8  = results(:,10);
table_results.BESS9  = results(:,11);
table_results.BESS10  = results(:,12);

file_name = newdir + 'cost.csv';
writetable(table_results, file_name,'Delimiter',',')  

fprintf("Ok - File printed - %s \n",  file_name)

%% Analysis 2

% Load simulation results
load("sim_nasheq_model.mat")

% Check the names of the _out variables loaded in the workspace. They
% are used here.

t_intervals =[0.25 1.5;
              1.5 3;
              3 4.5;
              4.5 6.0];

n_intervals = size(t_intervals,1);

results = zeros(n_intervals+1,4);

for k = 1:1:n_intervals

    tin = t_intervals(k,1); %Initial time
    tfi = t_intervals(k,2); %Final time

    t   = nm_out.w.Time; %Get the time vector
    tint= (t>=tin & t<tfi); %Get the time interval under analysis

    % States of the DERs
    x_pv1_g     = get_x_g_der(nm_out.x_pv1, tint);
    x_bess2_g   = get_x_g_der(nm_out.x_bess2, tint);
    x_bess3_g   = get_x_g_der(nm_out.x_bess3, tint);
    x_pv4_g     = get_x_g_der(nm_out.x_pv4, tint);
    x_pv5_g     = get_x_g_der(nm_out.x_pv5, tint);
    x_bess6_g   = get_x_g_der(nm_out.x_bess6, tint);
    x_pv7_g     = get_x_g_der(nm_out.x_pv7, tint);
    x_bess8_g   = get_x_g_der(nm_out.x_bess8, tint);
    x_bess9_g   = get_x_g_der(nm_out.x_bess9, tint);
    x_bess10_g  = get_x_g_der(nm_out.x_bess10, tint);

    % Integral of error
    w_g = get_w_g(nm_out.w, tint);

    % Full vector state of the deviation system
    x_g = [x_pv1_g; x_bess2_g; x_bess3_g; x_pv4_g; x_pv5_g; x_bess6_g; x_pv7_g; x_bess8_g; x_bess9_g; x_bess10_g; w_g] ;

    % Control actions
    u_pv1_g     = get_w_g(nm_out.u_pv1, tint);
    u_bess2_g   = get_w_g(nm_out.u_bess2, tint);
    u_bess3_g   = get_w_g(nm_out.u_bess3, tint);
    u_pv4_g     = get_w_g(nm_out.u_pv4, tint);
    u_pv5_g     = get_w_g(nm_out.u_pv5, tint);
    u_bess6_g   = get_w_g(nm_out.u_bess6, tint);
    u_pv7_g     = get_w_g(nm_out.u_pv7, tint);
    u_bess8_g   = get_w_g(nm_out.u_bess8, tint);
    u_bess9_g   = get_w_g(nm_out.u_bess9, tint);
    u_bess10_g  = get_w_g(nm_out.u_bess10, tint);

    % Compute Ji of the deviation system by formula and integration
    PV1.cost    = get_J_by_integration(tin, tfi, Ts, PV1.Q_aug, PV1.Ru, x_g, u_pv1_g);
    BESS2.cost  = get_J_by_integration(tin, tfi, Ts, BESS2.Q_aug, BESS2.Ru, x_g, u_bess2_g);  
    BESS3.cost  = get_J_by_integration(tin, tfi, Ts, BESS3.Q_aug, BESS3.Ru, x_g, u_bess3_g);
    PV4.cost    = get_J_by_integration(tin, tfi, Ts, PV4.Q_aug, PV4.Ru, x_g, u_pv4_g);
    PV5.cost    = get_J_by_integration(tin, tfi, Ts, PV5.Q_aug, PV5.Ru, x_g, u_pv5_g);
    BESS6.cost  = get_J_by_integration(tin, tfi, Ts, BESS6.Q_aug, BESS6.Ru, x_g, u_bess6_g);
    PV7.cost    = get_J_by_integration(tin, tfi, Ts, PV7.Q_aug, PV7.Ru, x_g, u_pv7_g);
    BESS8.cost  = get_J_by_integration(tin, tfi, Ts, BESS8.Q_aug, BESS8.Ru, x_g, u_bess8_g);
    BESS9.cost  = get_J_by_integration(tin, tfi, Ts, BESS9.Q_aug, BESS9.Ru, x_g, u_bess9_g);
    BESS10.cost = get_J_by_integration(tin, tfi, Ts, BESS10.Q_aug, BESS10.Ru, x_g, u_bess10_g);

    results(k,1) = tin;
    results(k,2) = tfi;
    results(k,3) = PV1.cost/scale_factor;
    results(k,4) = BESS2.cost/scale_factor;
    results(k,5) = BESS3.cost/scale_factor;
    results(k,6) = PV4.cost /scale_factor;
    results(k,7) = PV5.cost /scale_factor;
    results(k,8) = BESS6.cost/scale_factor;
    results(k,9) = PV7.cost/scale_factor;
    results(k,10) = BESS8.cost/scale_factor;
    results(k,11) =  BESS9.cost/scale_factor;
    results(k,12) = BESS10.cost/scale_factor;
end

results(end,1) = t_intervals(1,1);
results(end,2) = t_intervals(end,end);
results(end,3) = sum(results(:,3));
results(end,4) = sum(results(:,4));
results(end,5) = sum(results(:,5));
results(end,6) = sum(results(:,6));
results(end,7) = sum(results(:,7));
results(end,8) = sum(results(:,8));
results(end,9) = sum(results(:,9));
results(end,10) = sum(results(:,10));
results(end,11) = sum(results(:,11));
results(end,12) = sum(results(:,12));

table_results = table;
table_results.start_time = results(:,1);
table_results.end_time = results(:,2);
table_results.PV1  = results(:,3);
table_results.BESS2  = results(:,4);
table_results.BESS3  = results(:,5);
table_results.PV4  = results(:,6);
table_results.PV5  = results(:,7);
table_results.BESS6  = results(:,8);
table_results.PV7  = results(:,9);
table_results.BESS8  = results(:,10);
table_results.BESS9  = results(:,11);
table_results.BESS10  = results(:,12);

file_name = newdir + 'cost_by_integration.csv';
writetable(table_results, file_name,'Delimiter',',')  

fprintf("Ok - File printed - %s \n",  file_name)
