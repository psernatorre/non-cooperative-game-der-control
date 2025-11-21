clear

%% Get path to save results
mydir  = pwd;
idcs   = strfind(mydir,'/');
newdir = mydir(1:idcs(end)-1);
newdir = newdir + "/outputs/";

%% Export results
load("sim_droop_model.mat")

variables = ["PV1_P", "BESS2_P" , "Load_P", "Grid_P", "Grid_Preq"];
file_name = "power_balance.csv";
export_results_csv(dm_out, variables, newdir, file_name)

%% Exports controllers
load("state_space_and_controllers.mat")

file_name = 'controller.csv';

ders = ["PV1", "BESS2"];
nders = size(ders',1);
controller_summary = zeros(nders,1);

for k=1:nders
    controller_summary(k,1) =  eval(ders(k)).("droop");
end
writematrix([ders' controller_summary], newdir + file_name)
fprintf("Ok - File printed %s \n", newdir + file_name)

%% Export scope figure

file = "droop_model";
scopeName = 'Scope1';

print_scope_figure(file, scopeName, newdir, scopeName)
fprintf("Ok - File printed %s \n", newdir + scopeName)

close_system(file)
close all