function out = aircraft_dynamics_linear(varargin)
% aircraft_dynamics_linear
% This file implements "successive linearization at each controller call" for NL-MPC.
%
% It works as a TWO-USE function (same file, two call patterns):
%
%   Mode 1 (NMPC StateFcn):
%       dxdt = aircraft_dynamics_linear(x,u)
%     - Called MANY times by the NL-MPC optimizer during one sample.
%     - Uses a "frozen" linear model (A,B,E, x̄,ū,d̄, ẋ̄) computed previously.
%
%   Mode 2 (Update mode called ONCE per Ts from Simulink):
%       y = aircraft_dynamics_linear('update', t, x_meas)
%     - Called ONCE each controller step Ts using the *measured plant state*.
%     - It recomputes the linearization at the current operating point:
%           x̄ = x_meas
%           ū = [u1(t) u2(t) u3(t) u4(t)] from profile arrays/functions
%           d̄ = [T_ram(t) P_prop(t)] from parameters(t) and P_prop(t)
%     - Stores the frozen matrices and offsets in a persistent struct (LIN).
%
% INPUT vector definition used by NL-MPC:
%   u = [u1..u8]' = [P_batt P_gen m_dot m_dot_air T_ram P_prop Speed rho]'
% In THIS linearizer we only need:
%   - MVs:  u(1:4)
%   - MDs:  u(5:6) = [T_ram; P_prop]
% Speed and rho are used in controller constraints, not in this linearized model.
%
% HOW "linearize each controller call" is achieved:
%   - NOT by Ts inside this function
%   - BUT by Simulink calling: aircraft_dynamics_linear('update', t_k, x_k)
%     once every Ts using ZOH(Ts) + Digital Clock(Ts) + an update block.

persistent CONST LIN

% =====================================================================
% ===================== CONSTANTS (init once) =========================
% =====================================================================
% This block runs only the first time the function is called.
% Everything inside CONST is constant model data and does not change with time.
if isempty(CONST)

    % -------------------- Efficiencies --------------------
    % These parameters model losses through components.
    CONST.eta_b   = 0.9;   % battery efficiency
    CONST.eta_c   = 0.93;  % converter efficiency
    CONST.eta_g   = 0.9;   % generator efficiency
    CONST.eta_r   = 0.93;  % rectifier efficiency
    CONST.eta_bus = 0.99;  % bus efficiency
    CONST.eta_m   = 0.9;   % motor efficiency
    CONST.eta_i   = 0.93;  % inverter efficiency

    % -------------------- Heat capacities --------------------
    % Used in heat flow terms m_dot * cp * T.
    CONST.cp_tot = 3500;   % coolant effective heat capacity 
    CONST.cp_air = 1006;   % air heat capacity

    % -------------------- Areas / heat transfer coefficients --------------------
    % Used in conduction/heat-exchanger terms h*A*(T_hot - T_cold).
    CONST.h_cp  = 8500;
    CONST.h_hx  = 10500;
    CONST.h_ram = 17500;
    CONST.A_s   = 2.262;
    CONST.A_MD  = 3.534;
    CONST.A_hx  = 60;
    CONST.A_ram = 103;

    % -------------------- Incidence matrix M (12x20) --------------------
    % Graph-based modeling: maps heat/power flows to nodes.
    CONST.M = [ ...
        -1   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
         0  -1   1   0   0   0  -1   0   0   0   0   0   0   0   0   0   0   0   0   0; 
         0   0   0   1  -1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
         0   0  -1  -1   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0; 
         0   0   0   0   0   0   0   0   1  -1   0   0   0   0   0   0   0   0   0   0; 
         0   0   0   0   0   0   0  -1  -1   0   0   0   0   1   0   0   0   0   0   0; 
         0   0   0   0   0   0   1   0   0   0   0   0   0   0   0  -1   0   0   0   0; 
         0   0   0   0   0   0   0   0   0   0   0  -1   1   0   0   0   0   0   0   0; 
         0   0   0   0   0   0   0   0   0   0   0   0  -1  -1   1   0   0   0   0   0; 
         0   0   0   0   0   0   0   0   0   0   0   0   0   0  -1   1   1   0   0   0; 
         0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  -1   1   0   0; 
         0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  -1   1   0];

    % -------------------- Capacitance matrix C* (12x12) --------------------
    % Thermal "capacitances" (storage) for each node.
    CONST.C12 = diag([ ...
        245000, 1015000, 175000, 735000, ...
        280000, 1155000, 700000, 315000, ...
        1750000, 455000, 525000, 70420]);

    % -------------------- D vector (ram injection) --------------------
    % Indicates where the ram-air power Ps is injected in the node equations.
    CONST.Dvec       = zeros(12,1);
    CONST.Dvec(12,1) = 1;

    % -------------------- Mapping thermal nodes -> full state indices --------------------
    % The thermal model has 12 states; full model has 13 including SoC.
    % This map tells which full-state indices correspond to the 12 thermal states.
    CONST.idx_th2full = [1 2 4 5 6 7 8 9 10 11 12 13];

    % -------------------- SoC coefficient --------------------
    % dSoC/dt = k_soc * P_batt
    CONST.k_soc = -1000/(837*300*630*3600);
end

% =====================================================================
% ======================== UPDATE MODE =================================
% =====================================================================
% This branch is executed ONLY when the first input is a command string:
%   aircraft_dynamics_linear('update', t, x_meas)
if nargin >= 1 && (ischar(varargin{1}) || isstring(varargin{1}))

    cmd = char(varargin{1});

    % Enforce only supported command
    if ~strcmpi(cmd,'update')
        error("Unknown mode. Use aircraft_dynamics_linear('update',t,x).");
    end

    % t  : time (scalar), typically sampled at Ts using Digital Clock in Simulink
    % x0 : current plant state x(k) sampled at Ts using ZOH in Simulink
    t  = varargin{2};
    x0 = varargin{3}(:);

    % ---- Clamp time to mission duration (avoids calling profiles beyond their definition) ----
    tFinal = 4440;
    if t < 0, t = 0; end
    if t > tFinal, t = tFinal; end

    % -----------------------------------------------------------------
    % 1) OPERATING-POINT INPUTS (ū) FROM PROFILES (u1..u4)
    % -----------------------------------------------------------------
    % These are NOT the optimized MVs; they are fixed scheduled values used ONLY
    % to define the linearization operating point (Previous nonlinear only solution).
    u1b = u1(t);   % P_batt
    u2b = u2(t);   % P_gen
    u3b = u3(t);   % m_dot
    u4b = u4(t);   % m_dot_air
    ubar = [u1b; u2b; u3b; u4b];

    % -----------------------------------------------------------------
    % 2) OPERATING-POINT DISTURBANCES (d̄) FROM PROFILES
    % -----------------------------------------------------------------
    % parameters(t) returns [Temp, Speed, rho]. Only Temp (=T_ram) is used here.
    [T_ram, ~, ~] = parameters(t);

    % P_prop profile function (P_prop.m)
    P_prop_val = P_prop(t);

    % Store disturbance operating point d̄ = [T_ram; P_prop]
    dbar = [T_ram; P_prop_val];

    % -----------------------------------------------------------------
    % 3) BUILD THERMAL STATE VECTOR (12 states) FROM FULL STATE (13 states)
    % -----------------------------------------------------------------
    % Remove SoC (state 3) from the thermal state ordering used in the graph model.
    x_th = [ x0(1); x0(2); x0(4); x0(5); x0(6); x0(7); x0(8); x0(9); x0(10); x0(11); x0(12); x0(13) ];

    % Aliases for readability in heat-flow equations
    z1=x_th(1);  z2=x_th(2);  z3=x_th(3);  z4=x_th(4);
    z5=x_th(5);  z6=x_th(6);  z7=x_th(7);  z8=x_th(8);
    z9=x_th(9);  z10=x_th(10); z11=x_th(11); z12=x_th(12);

    % -----------------------------------------------------------------
    % 4) ANALYTIC JACOBIANS OF FLOW VECTOR P* (20 flows)
    % -----------------------------------------------------------------
    % JPx:  dP/dx_th   (20x12)
    % JPu:  dP/d[MVs]  (20x4)
    % JPp:  dP/dP_prop (20x1)
    JPx  = zeros(20,12);
    JPu  = zeros(20,4);
    JPp  = zeros(20,1);

    % Helpful combined coefficients
    hcAs = CONST.h_cp*CONST.A_s;
    hcAM = CONST.h_cp*CONST.A_MD;

    % ---- dP/dx_th entries (only where P depends on states) ----
    JPx(2,1)   =  hcAs;     JPx(2,2)  = -hcAs;         % P2 = h*A*(z1-z2)
    JPx(3,2)   =  u3b*CONST.cp_tot;                     % P4 = m_dot*cp*z2
    JPx(4,3)   =  hcAs;     JPx(4,4)  = -hcAs;         % P5 = h*A*(z3-z4)
    JPx(7,7)   =  u3b*CONST.cp_tot;                     % P8 = m_dot*cp*z7
    JPx(8,4)   =  u3b*CONST.cp_tot;                     % P9 = m_dot*cp*z4
    JPx(9,5)   =  hcAs;     JPx(9,6)  = -hcAs;         % P10 = h*A*(z5-z6)
    JPx(13,8)  =  hcAM;     JPx(13,9) = -hcAM;         % P14 = h*A*(z8-z9)
    JPx(14,6)  =  u3b*CONST.cp_tot;                     % P15 = m_dot*cp*z6
    JPx(15,9)  =  u3b*CONST.cp_tot;                     % P16 = m_dot*cp*z9
    JPx(16,10) =  u3b*CONST.cp_tot;                     % P17 = m_dot*cp*z10
    JPx(17,10) =  CONST.h_hx*CONST.A_hx;
    JPx(17,11) = -CONST.h_hx*CONST.A_hx;                % P18 = h*A*(z10-z11)
    JPx(18,11) =  CONST.h_ram*CONST.A_ram;
    JPx(18,12) = -CONST.h_ram*CONST.A_ram;              % P19 = h*A*(z11-z12)
    JPx(19,12) =  u4b*CONST.cp_air;                      % P20 = m_dot_air*cp_air*z12

    % ---- dP/d[MVs] entries ----
    % P1 depends on P_batt through efficiency equation
    k1 = (1 - CONST.eta_b*CONST.eta_c)/(CONST.eta_b*CONST.eta_c);
    JPu(1,1)  = k1;

    % Generator split P11 and P12 depend on P_gen (u2)
    JPu(10,2) = (1 - CONST.eta_g*CONST.eta_r);
    JPu(11,2) = (CONST.eta_g*CONST.eta_r);

    % Coolant mass flow m_dot (u3) affects flow terms that are m_dot*cp*T
    JPu(3,3)  = CONST.cp_tot*z2;
    JPu(7,3)  = CONST.cp_tot*z7;
    JPu(8,3)  = CONST.cp_tot*z4;
    JPu(14,3) = CONST.cp_tot*z6;
    JPu(15,3) = CONST.cp_tot*z9;
    JPu(16,3) = CONST.cp_tot*z10;

    % Ram-air mass flow (u4) affects P20 = m_dot_air*cp_air*z12
    JPu(19,4) = CONST.cp_air*z12;

    % ---- dP/dP_prop entries ----
    % P_prop propagates through bus/motor/inverter efficiency paths.
    k5  = (1 - CONST.eta_bus)/(CONST.eta_m*CONST.eta_i);
    k6  = 1/(CONST.eta_m*CONST.eta_i);
    k12 = (1 - CONST.eta_m*CONST.eta_i)/(CONST.eta_m*CONST.eta_i);
    JPp(5)  = k5;   % P6 term
    JPp(6)  = k6;   % P7 term
    JPp(12) = k12;  % P13 term
    JPp(20) = 1.0;  % P21 term

    % -----------------------------------------------------------------
    % 5) CONTINUOUS-TIME STATE-SPACE MATRICES FOR THERMAL SUBSYSTEM
    % -----------------------------------------------------------------
    % Graph dynamics: C * xdot = -M*P + D*Ps
    % After linearization: xdot = A_th*x + B_th*u + E_t*T_ram + E_p*P_prop + P0
    A1 = -CONST.M * JPx;    % 12x12
    B1 = -CONST.M * JPu;    % 12x4
    A2 = -CONST.M * JPp;    % 12x1

    % Divide by C (capacitance) to get continuous-time A,B,E matrices
    A_th = CONST.C12 \ A1;  % 12x12
    B_th = CONST.C12 \ B1;  % 12x4
    E_p  = CONST.C12 \ A2;  % 12x1 (P_prop input channel)
    Df   = CONST.C12 \ CONST.Dvec;  % 12x1 (injection shape)

    % T_ram enters through Ps = m_dot_air * cp_air * T_ram at the injection node
    E_t  = Df * (u4b * CONST.cp_air);  % 12x1 (T_ram input channel)

    % -----------------------------------------------------------------
    % 6) COMPUTE P* VECTOR AT OPERATING POINT (for affine offset P0)
    % -----------------------------------------------------------------
    % Pvec contains the 20 flow values at the current operating point.
    Pvec = zeros(20,1);
    Pvec(1)  = u1b*(1 - CONST.eta_b*CONST.eta_c)/(CONST.eta_b*CONST.eta_c);
    Pvec(2)  = CONST.h_cp*CONST.A_s*(z1 - z2);
    Pvec(3)  = u3b*CONST.cp_tot*z2;
    Pvec(4)  = CONST.h_cp*CONST.A_s*(z3 - z4);
    Pvec(5)  = P_prop_val*(1 - CONST.eta_bus)/(CONST.eta_m*CONST.eta_i);
    Pvec(6)  = P_prop_val/(CONST.eta_m*CONST.eta_i);
    Pvec(7)  = u3b*CONST.cp_tot*z7;
    Pvec(8)  = u3b*CONST.cp_tot*z4;
    Pvec(9)  = CONST.h_cp*CONST.A_s*(z5 - z6);
    Pvec(10) = (1 - CONST.eta_g*CONST.eta_r)*u2b;
    Pvec(11) = CONST.eta_g*CONST.eta_r*u2b;
    Pvec(12) = P_prop_val*(1 - CONST.eta_m*CONST.eta_i)/(CONST.eta_m*CONST.eta_i);
    Pvec(13) = CONST.h_cp*CONST.A_MD*(z8 - z9);
    Pvec(14) = u3b*CONST.cp_tot*z6;
    Pvec(15) = u3b*CONST.cp_tot*z9;
    Pvec(16) = u3b*CONST.cp_tot*z10;
    Pvec(17) = CONST.h_hx*CONST.A_hx*(z10 - z11);
    Pvec(18) = CONST.h_ram*CONST.A_ram*(z11 - z12);
    Pvec(19) = u4b*CONST.cp_air*z12;
    Pvec(20) = P_prop_val;

    % Offset ensures the linear model matches the nonlinear operating point exactly:
    % xdotbar = A*xbar + B*ubar + E*dbar + P0
    B2      = -CONST.M*Pvec - A1*x_th - A2*P_prop_val - B1*ubar;
    P0_base = CONST.C12 \ B2;

    % -----------------------------------------------------------------
    % 7) OPERATING-POINT DERIVATIVE (xdotbar)
    % -----------------------------------------------------------------
    xdot12 = A_th*x_th + B_th*ubar + E_t*T_ram + E_p*P_prop_val + P0_base;

    % Expand thermal 12-state derivative to the full 13-state derivative (adds SoC)
    A_full   = zeros(13,13);
    B_full   = zeros(13,4);
    E_full   = zeros(13,2);
    xdotbar  = zeros(13,1);

    % Map A_th, B_th, E_t, E_p into full state indices
    for i = 1:12
        fi = CONST.idx_th2full(i);
        xdotbar(fi)   = xdot12(i);
        B_full(fi,:)  = B_th(i,:);
        E_full(fi,1)  = E_t(i);    % T_ram channel
        E_full(fi,2)  = E_p(i);    % P_prop channel
        for j = 1:12
            fj = CONST.idx_th2full(j);
            A_full(fi,fj) = A_th(i,j);
        end
    end

    % SoC row: dSoC/dt depends only on P_batt (MV1)
    xdotbar(3)  = CONST.k_soc * u1b;
    B_full(3,1) = CONST.k_soc;

    % -----------------------------------------------------------------
    % 8) STORE THE "FROZEN" LINEAR MODEL FOR NMPC CALLS
    % -----------------------------------------------------------------
    LIN.ready   = true;      % indicates model is available
    LIN.xbar    = x0;        % operating point state x̄
    LIN.ubar    = ubar;      % operating point MV ū (from profiles)
    LIN.dbar    = dbar;      % operating point disturbance d̄ = [T_ram; P_prop]
    LIN.A       = A_full;    % frozen A matrix
    LIN.B       = B_full;    % frozen B matrix (4 MVs)
    LIN.E       = E_full;    % frozen E matrix (2 MDs used by linear model)
    LIN.xdotbar = xdotbar;   % derivative at operating point

    % Return dummy value so a Simulink MATLAB Function block can have an output.
    out = 0;
    return
end

% =====================================================================
% ======================== STATEFCN MODE ===============================
% =====================================================================
% This branch is executed when called as:
%   dxdt = aircraft_dynamics_linear(x,u)
% by nlmpc during optimization.

% Current candidate state used by optimizer (13x1)
x = varargin{1}(:);

% Current candidate input vector used by optimizer (contains MV and MD)
u = varargin{2}(:);

% Safety fallback: if 'update' was not called yet, initialize at t=0 using x
% (Normally Simulink calls update at t=0 before MPC executes.)
if isempty(LIN) || ~isfield(LIN,'ready') || ~LIN.ready
    aircraft_dynamics_linear('update', 0, x);
end

% Extract candidate MVs from optimizer:
uMV = u(1:4);

% Extract the two MDs used by this linear model:
%   d = [T_ram; P_prop]
d = u(5:6);

% Frozen delta-model prediction:
% dxdt = xdotbar + A*(x - xbar) + B*(uMV - ubar) + E*(d - dbar)
dxdt = LIN.xdotbar ...
     + LIN.A*(x   - LIN.xbar) ...
     + LIN.B*(uMV - LIN.ubar) ...
     + LIN.E*(d   - LIN.dbar);

out = dxdt;
end