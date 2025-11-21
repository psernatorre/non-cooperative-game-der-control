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

%% PV system 4 

% State space representation

PV4.A = [-263.094    -2.9559*10^4;
          1         0];

PV4.B = [1; 0];

PV4.C = [1.589 2.945*10^4];

% Kalman filter + LTR
PV4.kf_A = PV4.A - [+100 5000;0 0]*0.2;
PV4.kf_B = PV4.B - [1;0]*0.1;
PV4.kf_C = PV4.C + [1 1]*0.1;

PV4.kf_t  = 0.0001;
PV4.kf_N  = 1;
PV4.kf_Mo = eye(2,2);
PV4.kf_G = eye(2,2);

PV4.kf_W = (PV4.kf_t)^2*(PV4.kf_Mo) + (PV4.B)*(PV4.B)'; % Covariance of process noise
PV4.kf_V = (PV4.kf_t)^2*(PV4.kf_N) ; % Covariance of measurement noise
[PV4.kf_cov,~,~] = icare(PV4.kf_A', ...
                         [], ...
                         (PV4.kf_G)*PV4.kf_W*(PV4.kf_G)', ...
                         [], ...
                         [], ...
                         [], ...
                         -(PV4.kf_C)'*(PV4.kf_V)^(-1)*(PV4.kf_C)); % A*P1 + P1*A' + G*W*G' - P*C'*V^(-1)*C*P

PV4.kf_L = (PV4.kf_cov)*(PV4.kf_C)'*(PV4.kf_V)^(-1);

PV4.kf_As = PV4.kf_A - (PV4.kf_L)*(PV4.kf_C); 
PV4.kf_Bs = [PV4.kf_B PV4.kf_L]; 
PV4.kf_Cs = eye(2,2);
PV4.kf_Ds = zeros(2,2);

%% PV system 5 

% State space representation
PV5.A = [-263.094    -2.9559*10^4;
          1         0];

PV5.B = [1; 0];

PV5.C = [1.589 2.945*10^4];

% Kalman filter + LTR
PV5.kf_A = PV5.A - [+100 5000;0 0]*0.2;
PV5.kf_B = PV5.B - [1;0]*0.1;
PV5.kf_C = PV5.C + [1 1]*0.1;

PV5.kf_t  = 0.0001;
PV5.kf_N  = 1;
PV5.kf_Mo = eye(2,2);
PV5.kf_G = eye(2,2);

PV5.kf_W = (PV5.kf_t)^2*(PV5.kf_Mo) + (PV5.B)*(PV5.B)'; % Covariance of process noise
PV5.kf_V = (PV5.kf_t)^2*(PV5.kf_N) ; % Covariance of measurement noise
[PV5.kf_cov,~,~] = icare(PV5.kf_A', ...
                         [], ...
                         (PV5.kf_G)*PV5.kf_W*(PV5.kf_G)', ...
                         [], ...
                         [], ...
                         [], ...
                         -(PV5.kf_C)'*(PV5.kf_V)^(-1)*(PV5.kf_C)); % A*P1 + P1*A' + G*W*G' - P*C'*V^(-1)*C*P

PV5.kf_L = (PV5.kf_cov)*(PV5.kf_C)'*(PV5.kf_V)^(-1);

PV5.kf_As = PV5.kf_A - (PV5.kf_L)*(PV5.kf_C); 
PV5.kf_Bs = [PV5.kf_B PV5.kf_L]; 
PV5.kf_Cs = eye(2,2);
PV5.kf_Ds = zeros(2,2);

%% BESS 6

% State space representation

BESS6.A = [-258.087    -3.041e+04;
           1       0] ;

BESS6.B = [1 ;0];

BESS6.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS6.kf_A = BESS6.A - [+500 5000;0 0.5]*0.2;
BESS6.kf_B = BESS6.B - [1;0]*0.1;
BESS6.kf_C = BESS6.C + [1 1]*0.1;

BESS6.kf_t = 0.0001;
BESS6.kf_N = 1;
BESS6.kf_Mo = eye(2,2);
BESS6.kf_G = eye(2,2);

BESS6.kf_W = (BESS6.kf_t)^2*(BESS6.kf_Mo) + (BESS6.B)*(BESS6.B)'; % Covariance of process noise
BESS6.kf_V = (BESS6.kf_t)^2*(BESS6.kf_N) ; % Covariance of measurement noise
[BESS6.kf_cov,~,~] = icare(BESS6.kf_A', ...
                            [], ...
                            (BESS6.kf_G)*(BESS6.kf_W)*(BESS6.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS6.kf_C)'*(BESS6.kf_V)^(-1)*(BESS6.kf_C)); % A*P + P*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS6.kf_L = (BESS6.kf_cov)*(BESS6.kf_C)'*(BESS6.kf_V)^(-1);

BESS6.kf_As = BESS6.kf_A - (BESS6.kf_L)*(BESS6.kf_C); 
BESS6.kf_Bs = [BESS6.kf_B BESS6.kf_L]; 
BESS6.kf_Cs = eye(2,2);
BESS6.kf_Ds = zeros(2,2);

%% PV system 7 

% State space representation

PV7.A = [-263.094    -2.9559*10^4;
          1         0];

PV7.B = [1; 0];

PV7.C = [1.589 2.945*10^4];

% Kalman filter + LTR
PV7.kf_A = PV7.A - [+100 5000;0 0]*0.2;
PV7.kf_B = PV7.B - [1;0]*0.1;
PV7.kf_C = PV7.C + [1 1]*0.1;

PV7.kf_t  = 0.0001;
PV7.kf_N  = 1;
PV7.kf_Mo = eye(2,2);
PV7.kf_G = eye(2,2);

PV7.kf_W = (PV7.kf_t)^2*(PV7.kf_Mo) + (PV7.B)*(PV7.B)'; % Covariance of process noise
PV7.kf_V = (PV7.kf_t)^2*(PV7.kf_N) ; % Covariance of measurement noise
[PV7.kf_cov,~,~] = icare(PV7.kf_A', ...
                         [], ...
                         (PV7.kf_G)*PV7.kf_W*(PV7.kf_G)', ...
                         [], ...
                         [], ...
                         [], ...
                         -(PV7.kf_C)'*(PV7.kf_V)^(-1)*(PV7.kf_C)); % A*P1 + P1*A' + G*W*G' - P*C'*V^(-1)*C*P

PV7.kf_L = (PV7.kf_cov)*(PV7.kf_C)'*(PV7.kf_V)^(-1);

PV7.kf_As = PV7.kf_A - (PV7.kf_L)*(PV7.kf_C); 
PV7.kf_Bs = [PV7.kf_B PV7.kf_L]; 
PV7.kf_Cs = eye(2,2);
PV7.kf_Ds = zeros(2,2);

%% BESS 8

% State space representation

BESS8.A = [-258.087    -3.041e+04;
           1       0] ;

BESS8.B = [1 ;0];

BESS8.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS8.kf_A = BESS8.A - [+500 5000;0 0.5]*0.2;
BESS8.kf_B = BESS8.B - [1;0]*0.1;
BESS8.kf_C = BESS8.C + [1 1]*0.1;

BESS8.kf_t = 0.0001;
BESS8.kf_N = 1;
BESS8.kf_Mo = eye(2,2);
BESS8.kf_G = eye(2,2);

BESS8.kf_W = (BESS8.kf_t)^2*(BESS8.kf_Mo) + (BESS8.B)*(BESS8.B)'; % Covariance of process noise
BESS8.kf_V = (BESS8.kf_t)^2*(BESS8.kf_N) ; % Covariance of measurement noise
[BESS8.kf_cov,~,~] = icare(BESS8.kf_A', ...
                            [], ...
                            (BESS8.kf_G)*(BESS8.kf_W)*(BESS8.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS8.kf_C)'*(BESS8.kf_V)^(-1)*(BESS8.kf_C)); % A*P + P*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS8.kf_L = (BESS8.kf_cov)*(BESS8.kf_C)'*(BESS8.kf_V)^(-1);

BESS8.kf_As = BESS8.kf_A - (BESS8.kf_L)*(BESS8.kf_C); 
BESS8.kf_Bs = [BESS8.kf_B BESS8.kf_L]; 
BESS8.kf_Cs = eye(2,2);
BESS8.kf_Ds = zeros(2,2);

%% BESS 9

% State space representation

BESS9.A = [-258.087    -3.041e+04;
           1       0] ;

BESS9.B = [1 ;0];

BESS9.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS9.kf_A = BESS9.A - [+500 5000;0 0.5]*0.2;
BESS9.kf_B = BESS9.B - [1;0]*0.1;
BESS9.kf_C = BESS9.C + [1 1]*0.1;

BESS9.kf_t = 0.0001;
BESS9.kf_N = 1;
BESS9.kf_Mo = eye(2,2);
BESS9.kf_G = eye(2,2);

BESS9.kf_W = (BESS9.kf_t)^2*(BESS9.kf_Mo) + (BESS9.B)*(BESS9.B)'; % Covariance of process noise
BESS9.kf_V = (BESS9.kf_t)^2*(BESS9.kf_N) ; % Covariance of measurement noise
[BESS9.kf_cov,~,~] = icare(BESS9.kf_A', ...
                            [], ...
                            (BESS9.kf_G)*(BESS9.kf_W)*(BESS9.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS9.kf_C)'*(BESS9.kf_V)^(-1)*(BESS9.kf_C)); % A*P + P*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS9.kf_L = (BESS9.kf_cov)*(BESS9.kf_C)'*(BESS9.kf_V)^(-1);

BESS9.kf_As = BESS9.kf_A - (BESS9.kf_L)*(BESS9.kf_C); 
BESS9.kf_Bs = [BESS9.kf_B BESS9.kf_L]; 
BESS9.kf_Cs = eye(2,2);
BESS9.kf_Ds = zeros(2,2);

%% BESS 10

% State space representation

BESS10.A = [-258.087    -3.041e+04;
           1       0] ;

BESS10.B = [1 ;0];

BESS10.C = [9.712 3.039*10^4];

% Kalman filter + LTR
BESS10.kf_A = BESS10.A - [+500 5000;0 0.5]*0.2;
BESS10.kf_B = BESS10.B - [1;0]*0.1;
BESS10.kf_C = BESS10.C + [1 1]*0.1;

BESS10.kf_t = 0.0001;
BESS10.kf_N = 1;
BESS10.kf_Mo = eye(2,2);
BESS10.kf_G = eye(2,2);

BESS10.kf_W = (BESS10.kf_t)^2*(BESS10.kf_Mo) + (BESS10.B)*(BESS10.B)'; % Covariance of process noise
BESS10.kf_V = (BESS10.kf_t)^2*(BESS10.kf_N) ; % Covariance of measurement noise
[BESS10.kf_cov,~,~] = icare(BESS10.kf_A', ...
                            [], ...
                            (BESS10.kf_G)*(BESS10.kf_W)*(BESS10.kf_G)', ...
                            [], ...
                            [], ...
                            [], ...
                            -(BESS10.kf_C)'*(BESS10.kf_V)^(-1)*(BESS10.kf_C)); % A*P + P*A' + G*W*G' - P*C'*V^(-1)*C*P
BESS10.kf_L = (BESS10.kf_cov)*(BESS10.kf_C)'*(BESS10.kf_V)^(-1);

BESS10.kf_As = BESS10.kf_A - (BESS10.kf_L)*(BESS10.kf_C); 
BESS10.kf_Bs = [BESS10.kf_B BESS10.kf_L]; 
BESS10.kf_Cs = eye(2,2);
BESS10.kf_Ds = zeros(2,2);

%% System state-space representation

A = blkdiag(PV1.A, BESS2.A, BESS3.A, PV4.A, PV5.A, BESS6.A, ...
    PV7.A, BESS8.A, BESS9.A, BESS10.A);

B = blkdiag(PV1.B, BESS2.B, BESS3.B, PV4.B, PV5.B, BESS6.B, ...
    PV7.B, BESS8.B, BESS9.B, BESS10.B);

C = [PV1.C BESS2.C BESS3.C PV4.C PV5.C BESS6.C PV7.C BESS8.C BESS9.C BESS10.C];

%% Initial condition 

PV1.Pini = 1.0e6;
BESS2.Pini = 1.0e6;
BESS3.Pini = 1.0e6;
PV4.Pini = 1.0e6;
PV5.Pini = 1.0e6;
BESS6.Pini = 1.0e6;
PV7.Pini = 1.0e6;
BESS8.Pini = 1.0e6;
BESS9.Pini = 1.0e6;
BESS10.Pini = 1.0e6;

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
    "PV4", ...
    "PV5", ...
    "BESS6", ...
    "PV7",...
    "BESS8",...
    "BESS9",...
    "BESS10",...
    "Grid", ...
    "Load");






