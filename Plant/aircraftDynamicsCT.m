function dxdt = aircraftDynamicsCT(x,u)
%% aircraftDynamicsCT
% Nonlinear continuous-time plant model for the hybrid-electric aircraft EMS thermal network.
%
% Concept (high level):
% - This implements a graph-based thermal/power-flow model of the form:
%
%       C * x_dot = M_bar * P(x,u)  +  D * P_in(u)
%
%   where:
%     x      : 13x1 state vector (temperatures + SoC / energy state, depending on your definition)
%     u      : input vector (contains MVs and MDs; see below)
%     C      : 13x13 "capacitance" matrix (thermal storage)
%     M_bar  : incidence/connection matrix mapping flows to node balances
%     P      : vector of 21 nonlinear flow terms 
%     D      : injection selector 
%     P_in   : injected power 
%
% Output:
%   dxdt   : 13x1 state derivative (continuous-time)
%
% IMPORTANT:
% - This file is NONLINEAR PLANT used in Simulink (continuous dynamics).
% - It is different from aircraft_dynamics_linear.m, which is used as the NMPC prediction model.

%% These quadratic function coefficients were obtained from fitting quadratic
% set of data of charge efficiency vs temperature
%
% eta_b is modeled as a temperature-dependent efficiency based on battery cold plate temperature x(1).
alpha = -0.0001;  % coefficient for x(1)^2 term (too negative can cause numeric issues)
beta  = 0.01;     % coefficient for x(1) term  (too small can cause numeric issues)
gamma = 0.8807;   % constant offset term

% Battery efficiency as quadratic function of temperature (x(1))
eta_b = alpha * x(1).^2 + beta .* x(1) + gamma;

%% Efficiencies
% Other efficiencies are treated constant here.
% (eta_b is variable above)
% eta_b = 0.9;
eta_c   = 0.93;
eta_g   = 0.9;
eta_r   = 0.93;
eta_bus = 0.99;
eta_m   = 0.9;
eta_i   = 0.93;

%% Dimensions of CPs and HX obtained from the thesis, but remained unchanged
% after assuming mass reduction
%
% Heat-transfer areas (m^2)
A_s   = 2.262;
A_MD  = 3.534;
A_hx  = 60;
A_ram = 103;

% Heat-transfer coefficients (W/(m^2.K))
h_cp  = 8500;
h_hx  = 10500;
h_ram = 17500;

%% Thermal coolant and air properties
% Coolant cp is blended from water and thermal fluid values.
cp_water = 4200;                 % J/(kg.K) at 20C
cp_fl    = 2800;                 % J/(kg.K) at 20C
cp       = 0.5*cp_fl + 0.5*cp_water; % blended coolant cp

% Air cp (J/(kg.K))
cp_air = 1006;

%% Capacitance / inertia matrix C (13x13)
% Represents thermal storage for each node + one very large term for state 3.
% Here, C is diagonal (each state has its own capacity).
C =  [245000       0           0           0           0           0           0           0           0           0           0           0           0
       0     1015000           0           0           0           0           0           0           0           0           0           0           0
       0           0   680400000           0           0           0           0           0           0           0           0           0           0
       0           0           0      175000           0           0           0           0           0           0           0           0           0
       0           0           0           0      735000           0           0           0           0           0           0           0           0
       0           0           0           0           0      280000           0           0           0           0           0           0           0
       0           0           0           0           0           0     1155000           0           0           0           0           0           0
       0           0           0           0           0           0           0      700000           0           0           0           0           0
       0           0           0           0           0           0           0           0      315000           0           0           0           0
       0           0           0           0           0           0           0           0           0     1750000           0           0           0
       0           0           0           0           0           0           0           0           0           0      455000           0           0
       0           0           0           0           0           0           0           0           0           0           0      525000           0
       0           0           0           0           0           0           0           0           0           0           0           0       70420];

%% Incidence / connection matrix M_bar (13x21)
% Maps each of the 21 flow terms in P into the node balances.
% Each row corresponds to a node (state); each column corresponds to a flow term P(i).
M_bar = [1 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 -1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
         -1 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 -1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 -1 1 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 1 0 0 0 0 -1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 1 -1 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 -1 0 0 0 0 0 0 0 -1;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 -1 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 0];

%% Flow vector P (21x1)
% Each entry corresponds to a physical heat/power flow equation.
% The comments "eq1..eq21" label these flow terms.
P = [u(1)*(1-eta_b*eta_c)/(eta_b*eta_c); % eq1: 
    h_cp*A_s*(x(1)-x(2));                % eq2: 
    u(1);                                % eq3: 
    u(2)*cp*x(2);                        % eq4: 
    h_cp*A_s*(x(4)-x(5));                % eq5: 
    u(6)*(1-eta_bus)/(eta_m*eta_i);      % eq6: 
    u(6)/(eta_m*eta_i);                  % eq7: 
    u(2)*cp*x(8);                        % eq8: 
    u(2)*cp*x(5);                        % eq9: 
    h_cp*A_s*(x(6)-x(7));                % eq10: 
    (1-eta_g*eta_r)*u(5);                % eq11:
    eta_g*eta_r*u(5);                    % eq12: 
    u(6)*(1-eta_m*eta_i)/(eta_m*eta_i);  % eq13: 
    h_cp*A_MD*(x(9)-x(10));              % eq14:
    u(2)*cp*x(7);                        % eq15: 
    u(2)*cp*x(10);                       % eq16: 
    u(2)*cp*x(11);                       % eq17:
    h_hx*A_hx*(x(11)-x(12));             % eq18: 
    h_ram*A_ram*(x(12)-x(13));           % eq19: 
    u(3)*cp_air*x(13);                   % eq20: 
    u(6)];                               % eq21: 

%% Injection selector D (13x1)
% Indicates which node receives the injected power P_in.
D = [0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     0;
     1];

%% injected power P_in

P_in = u(4)*cp_air*u(5);

%% State derivative
% Solve for x_dot:
%   x_dot = C^{-1} * (M_bar*P + D*P_in)
%
% pinv(C) is used instead of inv(C) for numerical robustness.
dxdt = pinv(C) * M_bar * P + pinv(C) * D * P_in;

end