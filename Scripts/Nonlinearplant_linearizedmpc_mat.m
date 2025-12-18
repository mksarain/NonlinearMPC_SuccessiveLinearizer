%% Economic MPC Control of Energy Management System of Hybrid Electric Aircraft
%% Graph Based Modelling (C*X_DOT = -M*P + D*P_IN)
%
% Notes:
% - This script configures a nonlinear MPC (nlmpc) controller for the hybrid-electric aircraft
%   thermal/energy management system.
% - Prediction model used by MPC is the successive-linearized (frozen / updated each Ts) model
%   in aircraft_dynamics_linear.m.
% - The nonlinear plant (aircraftDynamicsCT.m) is executed in Simulink (subsystem/plant block).
% - Manipulated Variables (MVs) are optimized by MPC.
% - Measured Disturbances (MDs) are generated in Simulink through MATLAB Function blocks.
% - Successive linearization is updated at each controller call via:
%       aircraft_dynamics_linear('update', t, x_k)
%   where x_k is sampled from the continuous plant using ZOH(Ts).

clear
clc
close all

%% -------------------- Nonlinear MPC Design --------------------
% Create an nlmpc object with:
%   - 13 states
%   - 13 outputs (y=x assumption)
%   - 4 MVs (indices 1..4)
%   - 4 MDs (indices 5..8)
nlobj = nlmpc(13,13,'MV',[1 2 3 4],'MD',[5 6 7 8]);

% -------------------- State names --------------------
% Naming the states makes debugging, plotting, and Simulink signal mapping easier.
nlobj.States(1).Name  = 'Tw_batt';  % Battery Cold Plate Temperature
nlobj.States(2).Name  = 'Tfl_batt'; % Fluid in Battery Cold Plate Temperature 
nlobj.States(3).Name  = 'SOC';      % Battery State of Charge
nlobj.States(4).Name  = 'Tw_bus';   % Bus Cold Plate Temperature
nlobj.States(5).Name  = 'Tfl_bus';  % Fluid in Bus Cold Plate Temperature
nlobj.States(6).Name  = 'Tw_gen';   % Generator Cold Plate Temperature
nlobj.States(7).Name  = 'Tfl_gen';  % Fluid in Generator Cold Plate Temperature
nlobj.States(8).Name  = 'Ttank';    % Fluid in Coolant Tank Temperature
nlobj.States(9).Name  = 'Tw_MD';    % Motor Drive Cold Plate Temperature
nlobj.States(10).Name = 'Tfl_MD';   % Fluid in Motor Drive Cold Plate Temperature
nlobj.States(11).Name = 'Tfl1_HX';  % Fluid in Coolant Tank Cold Plate Temperature
nlobj.States(12).Name = 'Tw_HX';    % Coolant Tank Cold Plate Temperature
nlobj.States(13).Name = 'Tfl2_HX';  % Fluid in Ram air Cold Plate Temperature

% -------------------- MV names --------------------
% MVs are decision variables optimized by the controller each sample.
nlobj.MV(1).Name = 'P_batt';        % Battery Optimized Power (not capacity)
nlobj.MV(2).Name = 'P_gen';         % Generator Optimized Power (not capacity)
nlobj.MV(3).Name = 'dot_m';         % Mass flow of Coolant Optimized
nlobj.MV(4).Name = 'dot_m_air';     % Mass Flow of Ram Air Optimized

% -------------------- MD names --------------------
% Succesive Linearizer calls from .m and in simulink these are in matlab functions
% MDs are provided to the controller (measured disturbances) and are NOT optimized.
nlobj.MD(1).Name = 'T_ram';         % Calculated with speed and rho in parametrs.m   %% T_ram is Temperature of Ram Air 
nlobj.MD(2).Name = 'P_prop';        % Calculated as an array in P_prop.m             %% P_prop is Total Power Demand to be fullfilled by P_batt and P_gen
nlobj.MD(3).Name = 'Speed';         % Calculated with T_ram and rho in parametrs.m   %% Speed of Aircraft
nlobj.MD(4).Name = 'rho';           % Calculated with speed and T_ram in parametrs.m %% RHO IS ALTITUDE

%% -------------------- MPC horizons / sample time --------------------
%   - the Nonlinear MPC block sample time in Simulink
%   - the ZOH sample time used to sample the continuous plant state
Ts = 60;
nlobj.Ts = Ts;

% Prediction horizon p and control horizon m:
% - p steps are used to predict behavior and evaluate cost/constraints.
% - m steps are used to optimize the MV moves (after m, moves are held).
nlobj.PredictionHorizon = 5;
nlobj.ControlHorizon    = 3;

%% -------------------- Nonlinear Plant Model aircraftDynamicsCT.M CALLED IN SIMULINK IN SUBSYSTEM BLOCK --------------------
%% -------------------- Prediction model (Succesive Linearizer and Plant for only MPC) --------------------
% The MPC uses aircraft_dynamics_linear.m as its prediction model.
% IMPORTANT:
% - aircraft_dynamics_linear is NOT the nonlinear plant.
% - It is the frozen/successive-linearized prediction model that is updated each Ts
%   by calling aircraft_dynamics_linear('update',t,xk) from Simulink.
nlobj.Model.StateFcn = 'aircraft_dynamics_linear'; % .m script
nlobj.Model.IsContinuousTime = true;

%% -------------------- Initial conditions --------------------
%% Random guess
% u0 is your initial guess for the 4 MVs (only used by Simulink controller initialization)
u0 = [400000; 472620; 3; 6];

% x0 is the initial state vector (13x1)
x0 = [20; 20; 0.95; 20; 20; 20; 20; 20; 20; 20; 20; 20; 20];

%% -------------------- Bounds --------------------
% States
% nlobj.States(1).Min = 0;
% nlobj.States(1).Max = 60;
% nlobj.States(2).Min = 0;
% nlobj.States(2).Max = 80;

% SoC limits (hard bounds on the prediction model)
nlobj.States(3).Min = 0.2;
nlobj.States(3).Max = 1;

% nlobj.States(4).Min = 0;
% nlobj.States(4).Max = 90;
% nlobj.States(5).Min = 0;
% nlobj.States(5).Max = 80;
% nlobj.States(7).Min = 0;
% nlobj.States(7).Max = 90;
% nlobj.States(8).Min = 0;
% nlobj.States(8).Max = 80;
% nlobj.States(10).Min = 0;
% nlobj.States(10).Max = 80;
% nlobj.States(11).Min = 0;
% nlobj.States(11).Max = 90;
% nlobj.States(12).Min = 0;
% nlobj.States(12).Max = 80;

% MVs
P_ref = 300e3;

nlobj.MV(1).Min = 0;
% nlobj.MV(1).Max = 800e3;
% nlobj.MV(1).RateMax = 10e3;
% nlobj.MV(1).RateMaxECR = 0.2;
% nlobj.MV(1).RateMin = -25e3;
% nlobj.MV(1).RateMinECR = 0.2;

nlobj.MV(2).Min = 0;
% nlobj.MV(2).Max = 1.4 * P_ref;
% nlobj.MV(2).RateMin = -20e3;
% nlobj.MV(2).RateMinECR = 0.05;

nlobj.MV(3).Min = 0;
nlobj.MV(3).Max = 8;
nlobj.MV(3).RateMin = -0.2;
nlobj.MV(3).RateMinECR = 0.05;
% nlobj.MV(3).RateMax = 0.5;
% nlobj.MV(3).RateMaxECR = 0.05;

% nlobj.MV(4).RateMin = -0.5;
% nlobj.MV(4).RateMax = 0.25;
% nlobj.MV(4).RateMinECR = 0.05;
% nlobj.MV(4).RateMaxECR = 0.05;

%% -------------------- Custom Economic Cost + Constraints --------------------
% Custom economic stage cost (replaces the standard quadratic MPC cost)
nlobj.Optimization.CustomCostFcn = 'aircraftCostFcn'; % .m script
nlobj.Optimization.ReplaceStandardCost = true;

% Custom constraints:
% - aircraftEqCon: equality constraints (e.g., power balance)
% - aircraftInEqCon: inequality constraints (e.g., MV/state bounds with slack)
nlobj.Optimization.CustomEqConFcn   = 'aircraftEqCon';    % .m script
nlobj.Optimization.CustomIneqConFcn = 'aircraftInEqCon';  % .m script

%% -------------------- (NEW) Initialize frozen linearization at t=0 --------------------
% This initializes the persistent frozen model used by aircraft_dynamics_linear(x,u).
% At runtime, Simulink will repeatedly call the 'update' mode once per Ts using x(k).
aircraft_dynamics_linear('update', 0, x0);

%% -------------------- Simulink Model with Economic MPC Controller --------------------
% Open the Simulink model window
mdl = 'Nonlinearplant_linearizedmpc_sim';
open_system(mdl)

% Start simulation (equivalent to pressing "Run" in Simulink)
set_param(mdl,'SimulationCommand','start');