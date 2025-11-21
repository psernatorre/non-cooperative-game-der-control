
clear

%% Get path to save results
mydir  = pwd;
idcs   = strfind(mydir,'/');
newdir = mydir(1:idcs(end)-1);
newdir = newdir + "/outputs/";

%% Export results
load("sim_nasheq_model.mat")

variables = ["PV1_P", "BESS2_P" , "BESS3_P", "Load_P", "Grid_P", "Grid_Preq"];
file_name = "power_balance.csv";
export_results_csv(nm_out, variables, newdir, file_name)

%% Exports controllers
load("state_space_and_controllers.mat")

file_name = 'controller.csv';

ders = ["PV1", "BESS2", "BESS3"];
states_per_der = 2; %From SI framework
nders = size(ders',1);
controller_summary = zeros(nders,1 + nders*states_per_der);

for k=1:nders
    controller_summary(k,1:end-1) =  eval(ders(k)).("NE_kp");
    controller_summary(k,end) = eval(ders(k)).("NE_ki");
end
writematrix([ders' controller_summary], newdir + file_name)
fprintf("Ok - File printed %s \n", newdir + file_name)

%% Export scope figure

file = "nasheq_model";
scopeName = 'Scope1';

print_scope_figure(file, scopeName, newdir, scopeName)
fprintf("Ok - File printed %s \n", newdir + scopeName)

close_system(file)
close all
