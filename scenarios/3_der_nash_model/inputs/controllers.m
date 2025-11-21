clear 
clc

load("state_space.mat")

%% Objective function

PV1.Ru = 1; % Control weighted matrix of PV1 
BESS2.Ru = 3; % Control weighted matrix of BESS1
BESS3.Ru = 2; % Control weighted matrix of BESS2

PV1.Qs = blkdiag(0.5*(PV1.C'*PV1.C), 0.2*(BESS2.C'*BESS2.C), 0.1*(BESS3.C'*BESS3.C)) ;

BESS2.Qs = blkdiag(0.3*(PV1.C'*PV1.C), 0.5*(BESS2.C'*BESS2.C), 0.1*(BESS3.C'*BESS3.C)) ;

BESS3.Qs = blkdiag(0.1*(PV1.C'*PV1.C), 0.1*(BESS2.C'*BESS2.C), 0.8*(BESS3.C'*BESS3.C)) ;


%Augmented parameters of the objective function for deviation system
PV1.Q_aug = blkdiag(PV1.Qs, 225);
           
BESS2.Q_aug = blkdiag(BESS2.Qs, 225);

BESS3.Q_aug = blkdiag(BESS3.Qs, 144);

%Augmented state space representation to track input
A_aug = [A zeros(6,1);-C 0];
B_aug = blkdiag(B,0);
B1_aug = B_aug(:,1);
B2_aug = B_aug(:,2);
B3_aug = B_aug(:,3);

%Test controllability of the system
Co = ctrb(A_aug, B_aug);
rank(Co)

PV1.S = B1_aug*(PV1.Ru)^(-1)*B1_aug';
BESS2.S = B2_aug*(BESS2.Ru)^(-1)*B2_aug';
BESS3.S = B3_aug*(BESS3.Ru)^(-1)*B3_aug';

P = cell(3,1);
Ak = cell(3,1);

Ak{1} = A_aug;
Ak{2} = A_aug;
Ak{3} = A_aug;

for k=1:40

P{1} = icare(Ak{1},[],PV1.Q_aug,[],[],[],-PV1.S);
P{2} = icare(Ak{2},[],BESS2.Q_aug,[],[],[],-BESS2.S);
P{3} = icare(Ak{3},[],BESS3.Q_aug,[],[],[],-BESS3.S);

Ak{1} = A_aug - 0*PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3};
Ak{2} = A_aug - PV1.S*P{1} - 0*BESS2.S*P{2} - BESS3.S*P{3};
Ak{3} = A_aug - PV1.S*P{1} - BESS2.S*P{2} - 0*BESS3.S*P{3};

end

%Checking solutions of the coupled riccati equations
(A_aug - 0*PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3})'*P{1} + P{1}*(A_aug - 0*PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3}) - P{1}*PV1.S*P{1} + PV1.Q_aug 

(A_aug - PV1.S*P{1} - 0*BESS2.S*P{2} - BESS3.S*P{3})'*P{2} + P{2}*(A_aug - PV1.S*P{1} - 0*BESS2.S*P{2} - BESS3.S*P{3}) - P{2}*BESS2.S*P{2} + BESS2.Q_aug 

(A_aug - PV1.S*P{1} - BESS2.S*P{2} - 0*BESS3.S*P{3})'*P{3} + P{3}*(A_aug - PV1.S*P{1} - BESS2.S*P{2} - 0*BESS3.S*P{3}) - P{3}*BESS3.S*P{3} + BESS3.Q_aug 

% The following must be stable
eig(A_aug - PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} )

% Coupled Riccati equation solutions (to consider in objective function)

PV1.PRic = P{1};
BESS2.PRic = P{2};
BESS3.PRic = P{3};

% Controllers
F1 = -(PV1.Ru)^(-1)*B1_aug'*P{1};
F2 = -(BESS2.Ru)^(-1)*B2_aug'*P{2};
F3 = -(BESS3.Ru)^(-1)*B3_aug'*P{3};

PV1.NE_kp = F1(1:6);
PV1.NE_ki = F1(end);

BESS2.NE_kp = F2(1:6);
BESS2.NE_ki = F2(end);

BESS3.NE_kp = F3(1:6);
BESS3.NE_ki = F3(end);

%% Save data
save("state_space_and_controllers", ...
    "A", "B", "C", ...
    "PV1", ...
    "BESS2", ...
    "BESS3",...
    "Grid", ...
    "Load");

