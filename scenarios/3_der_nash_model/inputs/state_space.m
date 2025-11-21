clear
clc
%% PV system 1 

% State space representation
PV1.A = [-263.094    -2.9559*10^4;
          1         0];

PV1.B = [1; 0];

PV1.C = [1.589 2.945*10^4];

% Kalman filter + LTR
PV1.kf_A = PV1.A - [+100 5000;0 0]*0.2;
PV1.kf_B = PV1.B - [1;0]*0.1;
PV1.kf_C = PV1.C + [1 1]*0.1;

PV1.kf_t  = 0.0001;
PV1.kf_N  = 1;
PV1.kf_Mo = eye(2,2);
PV1.kf_G = eye(2,2);

PV1.kf_W = (PV1.kf_t)^2*(PV1.kf_Mo) + (PV1.B)*(PV1.B)'; % Covariance of process noise
PV1.kf_V = (PV1.kf_t)^2*(PV1.kf_N) ; % Covariance of measurement noise
[PV1.kf_cov,~,~] = icare(PV1.kf_A', ...
                         [], ...
                         (PV1.kf_G)*PV1.kf_W*(PV1.kf_G)', ...
                         [], ...
                         [], ...
                         [], ...
                         -(PV1.kf_C)'*(PV1.kf_V)^(-1)*(PV1.kf_C)); % A*P1 + P1*A' + G*W*G' - P*C'*V^(-1)*C*P

PV1.kf_L = (PV1.kf_cov)*(PV1.kf_C)'*(PV1.kf_V)^(-1);

PV1.kf_As = PV1.kf_A - (PV1.kf_L)*(PV1.kf_C); 
PV1.kf_Bs = [PV1.kf_B PV1.kf_L]; 
PV1.kf_Cs = eye(2,2);
PV1.kf_Ds = zeros(2,2);

%% BESS 2

% State space representation
BESS2.A = [-258.087    -3.041e+04;
           1       0] ;

BESS2.B = [1 ;0];

BESS2.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS2.kf_A = BESS2.A - [+500 5000;0 0.5]*0.2;
BESS2.kf_B = BESS2.B - [1;0]*0.1;
BESS2.kf_C = BESS2.C + [1 1]*0.1;

BESS2.kf_t = 0.0001;
BESS2.kf_N = 1;
BESS2.kf_Mo = eye(2,2);
BESS2.kf_G = eye(2,2);

BESS2.kf_W = (BESS2.kf_t)^2*(BESS2.kf_Mo) + (BESS2.B)*(BESS2.B)'; % Covariance of process noise
BESS2.kf_V = (BESS2.kf_t)^2*(BESS2.kf_N) ; % Covariance of measurement noise
[BESS2.kf_cov,~,~] = icare(BESS2.kf_A', ...
                            [], ...
                            (BESS2.kf_G)*(BESS2.kf_W)*(BESS2.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS2.kf_C)'*(BESS2.kf_V)^(-1)*(BESS2.kf_C)); % A*P1 + P1*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS2.kf_L = (BESS2.kf_cov)*(BESS2.kf_C)'*(BESS2.kf_V)^(-1);

BESS2.kf_As = BESS2.kf_A - (BESS2.kf_L)*(BESS2.kf_C); 
BESS2.kf_Bs = [BESS2.kf_B BESS2.kf_L]; 
BESS2.kf_Cs = eye(2,2);
BESS2.kf_Ds = zeros(2,2);

%% BESS 3

% State space representation

BESS3.A = [-258.087    -3.041e+04;
           1       0] ;

BESS3.B = [1 ;0];

BESS3.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS3.kf_A = BESS3.A - [+500 5000;0 0.5]*0.2;
BESS3.kf_B = BESS3.B - [1;0]*0.1;
BESS3.kf_C = BESS3.C + [1 1]*0.1;

BESS3.kf_t = 0.0001;
BESS3.kf_N = 1;
BESS3.kf_Mo = eye(2,2);
BESS3.kf_G = eye(2,2);

BESS3.kf_W = (BESS3.kf_t)^2*(BESS3.kf_Mo) + (BESS3.B)*(BESS3.B)'; % Covariance of process noise
BESS3.kf_V = (BESS3.kf_t)^2*(BESS3.kf_N) ; % Covariance of measurement noise
[BESS3.kf_cov,~,~] = icare(BESS3.kf_A', ...
                            [], ...
                            (BESS3.kf_G)*(BESS3.kf_W)*(BESS3.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS3.kf_C)'*(BESS3.kf_V)^(-1)*(BESS3.kf_C)); % A*P + P*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS3.kf_L = (BESS3.kf_cov)*(BESS3.kf_C)'*(BESS3.kf_V)^(-1);

BESS3.kf_As = BESS3.kf_A - (BESS3.kf_L)*(BESS3.kf_C); 
BESS3.kf_Bs = [BESS3.kf_B BESS3.kf_L]; 
BESS3.kf_Cs = eye(2,2);
BESS3.kf_Ds = zeros(2,2);

%% System state-space representation

A = blkdiag(PV1.A, BESS2.A, BESS3.A);

B = [ [PV1.B; zeros(4,1)] , [zeros(2,1); BESS2.B; zeros(2,1)], [zeros(4,1); BESS3.B]];

C = [ PV1.C BESS2.C BESS3.C];

%% Initial condition 

PV1.Pini = 6.0e6;
BESS2.Pini = 2.0e6;
BESS3.Pini = 2.0e6;

Load.P_1    = 14.0e6; % [W] Active power of load 1
Load.QL_1   = 0;      % [VAR] Reactive power of load 1
Load.Qc_1   = 0;      % [VAR] Reactive power of load 1

%% Power regulation request

Grid.Preq1  = 11.0e6; 
Grid.Time1  = 0.25;

Grid.Preq2  = -25.0e6; 
Grid.Time2  = 3;

%% Load perturbations

Load.sw2_closed = 1.5;       % [s] time to close the breaker
Load.P_2    = 4e6;  % [W] Active power of load 2
Load.QL_2   = 0;      % [VAR] Reactive power of load 2
Load.Qc_2   = 0;      % [VAR] Reactive power of load 2

Load.sw3_closed = 3.00; % [s] time to close the breaker
Load.sw3_open = 4.50; % [s] time to open the breaker
Load.P_3    = 5.0e6;
Load.QL_3   = 0;      % [VAR] Reactive power of load 3
Load.Qc_3   = 0;      % [VAR] Reactive power of load 3

%% Save data (to be used by simulation)
save("state_space", ...
    "A", "B", "C", ...
    "PV1", ...
    "BESS2", ...
    "BESS3", ...
    "Grid", ...
    "Load");





