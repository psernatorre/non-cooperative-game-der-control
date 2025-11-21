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

    % Integral of error
    w_g = get_w_g(nm_out.w, tint);

    % Full vector state of the deviation system
    x_g = [x_pv1_g; x_bess2_g; x_bess3_g; w_g] ;

    % Control actions
    u_pv1_g = get_w_g(nm_out.u_pv1, tint);
    u_bess2_g = get_w_g(nm_out.u_bess2, tint);
    u_bess3_g = get_w_g(nm_out.u_bess3, tint);

    % Compute Ji of the deviation system by formula and integration
    PV1.cost = x_g(:,1)'*PV1.PRic*x_g(:,1);

    BESS2.cost = x_g(:,1)'*BESS2.PRic*x_g(:,1);

    BESS3.cost = x_g(:,1)'*BESS3.PRic*x_g(:,1);

    results(k,1) = tin;
    results(k,2) = tfi;
    results(k,3:5) = [PV1.cost, BESS2.cost, BESS3.cost]/scale_factor;
end
results(end,1) = t_intervals(1,1);
results(end,2) = t_intervals(end,end);
for k=1:3
    results(end,k+2) = sum(results(:,k+2));
end
table_results = table;
table_results.start_time = results(:,1);
table_results.end_time = results(:,2);
table_results.PV1  = results(:,3);
table_results.BESS2  = results(:,4);
table_results.BESS3  = results(:,5);


file_name = newdir + 'cost.csv';
writetable(table_results, file_name,'Delimiter',',')  

fprintf("Ok - File printed - %s \n",  file_name)

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

    % Integral of error
    w_g = get_w_g(nm_out.w, tint);

    % Full vector state of the deviation system
    x_g = [x_pv1_g; x_bess2_g; x_bess3_g; w_g] ;

    % Control actions
    u_pv1_g = get_w_g(nm_out.u_pv1, tint);
    u_bess2_g = get_w_g(nm_out.u_bess2, tint);
    u_bess3_g = get_w_g(nm_out.u_bess3, tint);

    % Compute Ji of the deviation system by formula and integration
    PV1.cost = get_J_by_integration(tin, tfi, Ts, PV1.Q_aug, PV1.Ru, x_g, u_pv1_g);

    BESS2.cost = get_J_by_integration(tin, tfi, Ts, BESS2.Q_aug, BESS2.Ru, x_g, u_bess2_g);
    
    BESS3.cost = get_J_by_integration(tin, tfi, Ts, BESS3.Q_aug, BESS3.Ru, x_g, u_bess3_g);

    results(k,1) = tin;
    results(k,2) = tfi;
    results(k,3:5) = [PV1.cost, BESS2.cost, BESS3.cost]/scale_factor;
end
results(end,1) = t_intervals(1,1);
results(end,2) = t_intervals(end,end);
for k=1:3
    results(end,k+2) = sum(results(:,k+2));
end
table_results = table;
table_results.start_time = results(:,1);
table_results.end_time = results(:,2);
table_results.PV1  = results(:,3);
table_results.BESS2  = results(:,4);
table_results.BESS3  = results(:,5);

file_name = newdir + 'cost_by_integration.csv';
writetable(table_results, file_name,'Delimiter',',')  

fprintf("Ok - File printed - %s \n",  file_name)