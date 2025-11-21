clear 
clc

load("state_space.mat")

%% Objective function

PV1.Ru = 1; % Control weighted matrix of PV1 
BESS2.Ru = 1; % Control weighted matrix of BESS1

PV1.Qs = [5*(PV1.C'*PV1.C) zeros(2,2); 
            zeros(2,2)  2*(BESS2.C'*BESS2.C)];

BESS2.Qs = [3*(PV1.C'*PV1.C) zeros(2,2); 
            zeros(2,2)  5*(BESS2.C'*BESS2.C)];

%Augmented parameters of the objective function for deviation system
PV1.Q_aug = [PV1.Qs zeros(4,1);0 0 0 0 125];
BESS2.Q_aug = [BESS2.Qs zeros(4,1);0 0 0 0 289];

%Augmented state spare representation to track input
A_aug = [A zeros(4,1);-C 0];
B1_aug = [PV1.B; zeros(2,1); 0];
B2_aug = [zeros(2,1); BESS2.B ; 0];

%Test controllability of the system
Co = ctrb(A_aug, [B1_aug B2_aug]);
rank(Co)

%% Droop controllers

PV1.droop = +1;
BESS2.droop = +17/13;

%% Save data
save("state_space_and_controllers", ...
    "A", "B", "C", ...
    "PV1", ...
    "BESS2", ...
    "Grid", ...
    "Load");

