clear
clc

%% General parameters
Ts       = 0.0001;
paramStruct.StopTime = '6';

%% Load space-state representation, observers and controllers

load("state_space_and_controllers")

%% System (grid) Parameters
% -------------------------------------------------------------------------
ts      = Ts;               % [sec] Time step
fBase   = 60;               % [Hz] system base frequency
system.Vbase    = 10e3;    % [V] system base voltage

%% PV1 Parameters
% -------------------------------------------------------------------------
PV1.Pref        = PV1.Pini;     % [W] real power output (initial condition)
PV1.Qref        = 0e6;     % [VA] reactive power output reference (negative is capacitive)
PV1.Vbase       = 690;      % [V] device voltage base
PV1.VbaseDC     = 600*30;   % [v] for the capacitor
%
PV1.rating      = 60e6;     % [VA] nameplate rating
%
PV1.irr.rating  = 1000;     % [W/m^2] rated irradiance input for rated output
PV1.irr.ref     = 1000;     % [W/m^2] used as constant irradiance input

%% BESS1 Parameters
% -------------------------------------------------------------------------
BESS2.Pref       = BESS2.Pini;   % [W] real power output (initial condition)
BESS2.rating     = 10e6;        % [VA] Inverter rating
BESS2.Qref       = -1e6;        % [VA] Reference Reactive Power
BESS2.Vbase      = 480;         % [V] Inverter voltage
BESS2.VbaseDC    = 1000;        % DC voltage at the input of the inverter

BESS2.maxContPower   = BESS2.rating ;   % [VA] Maximum Continuos Power
BESS2.capacity       = 1221;            % [kWh] BESS Capacity - Note: This value needs to be verified
BESS2.efficiency     = 85;              % [%] BESS Efficiency
BESS2.initSOC        = 50;              % [%] Initial SoC
BESS2.minSOC         = 10;              % [%] Minimum SoC limit
BESS2.maxSOC         = 90;              % [%] Maximum SoC limit

%% Grid parameters

Grid.V_pu   = 1.0; % [pu] Voltage set to the grid.
Grid.V_base = system.Vbase;

%% Load parameters

Load.Vbase  = system.Vbase;

%% Model simulation using droop controller

file = "droop_model.slx";

open_system(file);

dm_out = sim(file, paramStruct);

save("sim_droop_model", "dm_out");

