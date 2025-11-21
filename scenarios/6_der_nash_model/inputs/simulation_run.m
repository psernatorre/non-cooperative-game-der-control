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

%% BESS2 Parameters
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

%% BESS3 Parameters
% -------------------------------------------------------------------------
BESS3.Pref       = BESS3.Pini;   % [W] real power output (initial condition)
BESS3.rating     = 10e6;        % [VA] Inverter rating
BESS3.Qref       = -1e6;        % [VA] Reference Reactive Power
BESS3.Vbase      = 480;         % [V] Inverter voltage
BESS3.VbaseDC    = 1000;        % DC voltage at the input of the inverter

BESS3.maxContPower   = BESS3.rating ;   % [VA] Maximum Continuos Power
BESS3.capacity       = 1221;            % [kWh] BESS Capacity - Note: This value needs to be verified
BESS3.efficiency     = 85;              % [%] BESS Efficiency
BESS3.initSOC        = 50;              % [%] Initial SoC
BESS3.minSOC         = 10;              % [%] Minimum SoC limit
BESS3.maxSOC         = 90;              % [%] Maximum SoC limit

%% PV4 Parameters
% -------------------------------------------------------------------------
PV4.Pref        = PV4.Pini;     % [W] real power output (initial condition)
PV4.Qref        = 0e6;     % [VA] reactive power output reference (negative is capacitive)
PV4.Vbase       = 690;      % [V] device voltage base
PV4.VbaseDC     = 600*30;   % [v] for the capacitor
%
PV4.rating      = 60e6;     % [VA] nameplate rating
%
PV4.irr.rating  = 1000;     % [W/m^2] rated irradiance input for rated output
PV4.irr.ref     = 1000;     % [W/m^2] used as constant irradiance input

%% PV5 Parameters
% -------------------------------------------------------------------------
PV5.Pref        = PV5.Pini;     % [W] real power output (initial condition)
PV5.Qref        = 0e6;     % [VA] reactive power output reference (negative is capacitive)
PV5.Vbase       = 690;      % [V] device voltage base
PV5.VbaseDC     = 600*30;   % [v] for the capacitor
%
PV5.rating      = 60e6;     % [VA] nameplate rating
%
PV5.irr.rating  = 1000;     % [W/m^2] rated irradiance input for rated output
PV5.irr.ref     = 1000;     % [W/m^2] used as constant irradiance input

%% BESS6 Parameters
% -------------------------------------------------------------------------
BESS6.Pref       = BESS6.Pini;   % [W] real power output (initial condition)
BESS6.rating     = 10e6;        % [VA] Inverter rating
BESS6.Qref       = -1e6;        % [VA] Reference Reactive Power
BESS6.Vbase      = 480;         % [V] Inverter voltage
BESS6.VbaseDC    = 1000;        % DC voltage at the input of the inverter

BESS6.maxContPower   = BESS6.rating ;   % [VA] Maximum Continuos Power
BESS6.capacity       = 1221;            % [kWh] BESS Capacity - Note: This value needs to be verified
BESS6.efficiency     = 85;              % [%] BESS Efficiency
BESS6.initSOC        = 50;              % [%] Initial SoC
BESS6.minSOC         = 10;              % [%] Minimum SoC limit
BESS6.maxSOC         = 90;              % [%] Maximum SoC limit

%% Grid parameters

Grid.V_pu   = 1.0; % [pu] Voltage set to the grid.
Grid.V_base = system.Vbase;

%% Load parameters

Load.Vbase  = system.Vbase;

%% Simulation using controller

file = "nasheq_model.slx";

open_system(file);

nm_out = sim(file, paramStruct);

save("sim_nasheq_model", "nm_out");
