clear 
clc

load("state_space.mat")

%% Objective function

PV1.Ru = 1; % Control weighted matrix of PV1 
BESS2.Ru = 4; % Control weighted matrix of BESS1

PV1.Qs = [0.5*(PV1.C'*PV1.C) zeros(2,2); 
            zeros(2,2)  0.2*(BESS2.C'*BESS2.C)];

BESS2.Qs = [0.3*(PV1.C'*PV1.C) zeros(2,2); 
            zeros(2,2)  0.5*(BESS2.C'*BESS2.C)];

%Augmented parameters of the objective function for deviation system
PV1.Q_aug = [PV1.Qs zeros(4,1);0 0 0 0 100];
BESS2.Q_aug = [BESS2.Qs zeros(4,1);0 0 0 0 36];

%Augmented state spare representation to track input
A_aug = [A zeros(4,1);-C 0];
B1_aug = [PV1.B; zeros(2,1); 0];
B2_aug = [zeros(2,1); BESS2.B ; 0];

%Test controllability of the system
Co = ctrb(A_aug, [B1_aug B2_aug]);
rank(Co)

PV1.S = B1_aug*PV1.Ru^(-1)*B1_aug';
BESS2.S = B2_aug*BESS2.Ru^(-1)*B2_aug';

P = cell(2,1);
Ak = cell(2,1);

Ak{1} = A_aug;
Ak{2} = A_aug;

for k=1:40

P{1} = icare(Ak{1},[],PV1.Q_aug,[],[],[],-PV1.S);
P{2} = icare(Ak{2},[],BESS2.Q_aug,[],[],[],-BESS2.S);

Ak{1} = A_aug - 0*PV1.S*P{1} - BESS2.S*P{2};
Ak{2} = A_aug - PV1.S*P{1} - 0*BESS2.S*P{2};

end

%Checking solutions of the coupled riccati equations
(A_aug - 0*PV1.S*P{1} - BESS2.S*P{2})'*P{1} + P{1}*(A_aug - 0*PV1.S*P{1} - BESS2.S*P{2}) - P{1}*PV1.S*P{1}+ PV1.Q_aug 

eig(A_aug - PV1.S*P{1} - BESS2.S*P{2})

(A_aug - PV1.S*P{1} - 0*BESS2.S*P{2})'*P{2} + P{2}*(A_aug - PV1.S*P{1} - 0*BESS2.S*P{2}) - P{2}*BESS2.S*P{2}+ BESS2.Q_aug 

% Coupled Riccati equation solutions (to consider in objective function)

PV1.PRic = P{1};
BESS2.PRic = P{2};

% Controllers
F1 = -(PV1.Ru)^(-1)*B1_aug'*P{1};
F2 = -(BESS2.Ru)^(-1)*B2_aug'*P{2};

PV1.NE_kp = F1(1:4);
PV1.NE_ki = F1(5);

BESS2.NE_kp = F2(1:4);
BESS2.NE_ki = F2(5);

%% Save data
save("state_space_and_controllers", ...
    "A", "B", "C", ...
    "PV1", ...
    "BESS2", ...
    "Grid", ...
    "Load");

