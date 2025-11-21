clear 
clc

load("state_space.mat")

%% Objective function

PV1.Ru = 1; % Control weighted matrix of PV1 
BESS2.Ru = 1; % Control weighted matrix of BESS2
BESS3.Ru = 2; % Control weighted matrix of BESS3
PV4.Ru = 0.5; % Control weighted matrix of PV4
PV5.Ru = 0.5; % Control weighted matrix of PV5
BESS6.Ru = 4; % Control weighted matrix of BESS6

PV1.Qs = blkdiag(5*(PV1.C'*PV1.C), 2*(BESS2.C'*BESS2.C), 1*(BESS3.C'*BESS3.C), 1*(PV4.C'*PV4.C), 1*(PV5.C'*PV5.C), 1*(BESS6.C'*BESS6.C)) ;

BESS2.Qs = blkdiag(3*(PV1.C'*PV1.C), 5*(BESS2.C'*BESS2.C), 1*(BESS3.C'*BESS3.C), 1*(PV4.C'*PV4.C), 1*(PV5.C'*PV5.C), 1*(BESS6.C'*BESS6.C)) ;

BESS3.Qs = blkdiag(1*(PV1.C'*PV1.C), 1*(BESS2.C'*BESS2.C), 5*(BESS3.C'*BESS3.C), 0.9*(PV4.C'*PV4.C), 1*(PV5.C'*PV5.C), 1*(BESS6.C'*BESS6.C)) ;

PV4.Qs = blkdiag(1*(PV1.C'*PV1.C), 1*(BESS2.C'*BESS2.C), 3.5*(BESS3.C'*BESS3.C), 4*(PV4.C'*PV4.C), 3*(PV5.C'*PV5.C), 2*(BESS6.C'*BESS6.C)) ;

PV5.Qs = blkdiag(1*(PV1.C'*PV1.C), 2*(BESS2.C'*BESS2.C), 1*(BESS3.C'*BESS3.C), 1*(PV4.C'*PV4.C), 2.5*(PV5.C'*PV5.C), 1*(BESS6.C'*BESS6.C)) ;

BESS6.Qs = blkdiag(1*(PV1.C'*PV1.C), 2*(BESS2.C'*BESS2.C), 0.4*(BESS3.C'*BESS3.C), 1*(PV4.C'*PV4.C), 0.1*(PV5.C'*PV5.C), 5*(BESS6.C'*BESS6.C)) ;


%Augmented parameters of the objective function for deviation system
PV1.Q_aug = blkdiag(PV1.Qs, 400);
           
BESS2.Q_aug = blkdiag(BESS2.Qs, 225);

BESS3.Q_aug = blkdiag(BESS3.Qs, 279);

PV4.Q_aug = blkdiag(PV4.Qs, 625);

PV5.Q_aug = blkdiag(PV5.Qs, 324);

BESS6.Q_aug = blkdiag(BESS6.Qs, 100);


%Augmented state space representation to track input
A_aug = [A zeros(12,1);-C 0];
B_aug = blkdiag(B,0);
B1_aug = B_aug(:,1);
B2_aug = B_aug(:,2);
B3_aug = B_aug(:,3);
B4_aug = B_aug(:,4);
B5_aug = B_aug(:,5);
B6_aug = B_aug(:,6);

%Test controllability of the system
Co = ctrb(A_aug, B_aug);
rank(Co)

PV1.S = B1_aug*(PV1.Ru)^(-1)*B1_aug';
BESS2.S = B2_aug*(BESS2.Ru)^(-1)*B2_aug';
BESS3.S = B3_aug*(BESS3.Ru)^(-1)*B3_aug';
PV4.S = B4_aug*(PV4.Ru)^(-1)*B4_aug';
PV5.S = B5_aug*(PV5.Ru)^(-1)*B5_aug';
BESS6.S = B6_aug*(BESS6.Ru)^(-1)*B6_aug';


P = cell(6,1);
Ak = cell(6,1);

Ak{1} = A_aug;
Ak{2} = A_aug;
Ak{3} = A_aug;
Ak{4} = A_aug;
Ak{5} = A_aug;
Ak{6} = A_aug;

for k=1:90

P{1} = icare(Ak{1},[],PV1.Q_aug,[],[],[],-PV1.S);
P{2} = icare(Ak{2},[],BESS2.Q_aug,[],[],[],-BESS2.S);
P{3} = icare(Ak{3},[],BESS3.Q_aug,[],[],[],-BESS3.S);
P{4} = icare(Ak{4},[],PV4.Q_aug,[],[],[],-PV4.S);
P{5} = icare(Ak{5},[],PV5.Q_aug,[],[],[],-PV5.S);
P{6} = icare(Ak{6},[],BESS6.Q_aug,[],[],[],-BESS6.S);


Ak{1} = A_aug - 0*PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} - PV4.S*P{4} - PV5.S*P{5} - BESS6.S*P{6};
Ak{2} = A_aug - PV1.S*P{1} - 0*BESS2.S*P{2} - BESS3.S*P{3} - PV4.S*P{4} - PV5.S*P{5} - BESS6.S*P{6};
Ak{3} = A_aug - PV1.S*P{1} - BESS2.S*P{2} - 0*BESS3.S*P{3} - PV4.S*P{4} - PV5.S*P{5} - BESS6.S*P{6};
Ak{4} = A_aug - PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} - 0*PV4.S*P{4} - PV5.S*P{5} - BESS6.S*P{6};
Ak{5} = A_aug - PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} - PV4.S*P{4} - 0*PV5.S*P{5} - BESS6.S*P{6};
Ak{6} = A_aug - PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} - PV4.S*P{4} - PV5.S*P{5} - 0*BESS6.S*P{6};

end

%Checking solutions of the coupled riccati equations
Ak{1}'*P{1} + P{1}*Ak{1} - P{1}*PV1.S*P{1} + PV1.Q_aug 

Ak{2}'*P{2} + P{2}*Ak{2} - P{2}*BESS2.S*P{2} + BESS2.Q_aug 

Ak{3}'*P{3} + P{3}*Ak{3} - P{3}*BESS3.S*P{3} + BESS3.Q_aug 

Ak{4}'*P{4} + P{4}*Ak{4} - P{4}*PV4.S*P{4} + PV4.Q_aug 

Ak{5}'*P{5} + P{5}*Ak{5} - P{5}*PV5.S*P{5} + PV5.Q_aug 

Ak{6}'*P{6} + P{6}*Ak{6} - P{6}*BESS6.S*P{6} + BESS6.Q_aug 


% The following must be stable
eig(A_aug - PV1.S*P{1} - BESS2.S*P{2} - BESS3.S*P{3} - PV4.S*P{4} - PV5.S*P{5} - BESS6.S*P{6})

% Coupled Riccati equation solutions (to consider in objective function)

PV1.PRic = P{1};
BESS2.PRic = P{2};
BESS3.PRic = P{3};
PV4.PRic = P{4};
PV5.PRic = P{5};
BESS6.PRic = P{6};

% Controllers
F1 = -(PV1.Ru)^(-1)*B1_aug'*P{1};
F2 = -(BESS2.Ru)^(-1)*B2_aug'*P{2};
F3 = -(BESS3.Ru)^(-1)*B3_aug'*P{3};
F4 = -(PV4.Ru)^(-1)*B4_aug'*P{4};
F5 = -(PV5.Ru)^(-1)*B5_aug'*P{5};
F6 = -(BESS6.Ru)^(-1)*B6_aug'*P{6};

PV1.NE_kp = F1(1:12);
PV1.NE_ki = F1(end);

BESS2.NE_kp = F2(1:12);
BESS2.NE_ki = F2(end);

BESS3.NE_kp = F3(1:12);
BESS3.NE_ki = F3(end);

PV4.NE_kp = F4(1:12);
PV4.NE_ki = F4(end);

PV5.NE_kp = F5(1:12);
PV5.NE_ki = F5(end);

BESS6.NE_kp = F6(1:12);
BESS6.NE_ki = F6(end);


%% Save data
save("state_space_and_controllers", ...
    "A", "B", "C", ...
    "PV1", ...
    "BESS2", ...
    "BESS3",...
    "PV4", ...
    "PV5", ...
    "BESS6",...
    "Grid", ...
    "Load");

