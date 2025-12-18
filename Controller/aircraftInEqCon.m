function cineq = aircraftInEqCon(X, U, e, data)
% aircraftInEqCon
% Custom inequality constraint function for nlmpc.
%
% Purpose:
%   Builds a vector of inequality constraints of the form:
%       cineq <= 0
%   for all prediction steps over the horizon.
%
% This function enforces:
%   1) Bounds on MV4 (dot_m_air) based on air density and aircraft speed:
%        U4_Lb <= U4 <= U4_Ub
%      where:
%        U4_Lb = rho * V * A_c
%        U4_Ub = rho * V * A_c + dot_m_fan
%
%   2) Soft lower/upper bounds on multiple temperature states using slack e:
%        (LowerBound - e) <= Xi <= (UpperBound + e)
%
% Inputs (nlmpc convention):
%   X    : predicted state trajectory, typically (p+1)-by-Nx (here Nx=13)
%   U    : predicted input trajectory, typically (p+1)-by-Nu (here Nu=8)
%   e    : slack variable (scalar >= 0) to soften constraints
%   data : struct containing PredictionHorizon, MVIndex, MDIndex
%
% Output:
%   cineq: stacked inequality constraints vector. nlmpc enforces cineq <= 0.

    % -------------------- Parameters for U4 bounds --------------------
    % A_c: capture/duct cross-sectional area term in the mass flow relation
    A_c = 0.1;

    % dot_m_fan: additional allowable mass flow provided by fan (offset)
    dot_m_fan = 5;

    % -------------------- Horizon length --------------------
    p = data.PredictionHorizon;

    % -------------------- Extract MV4 trajectory (dot_m_air) --------------------
    % U4 is MV4 for steps 1..p
    U4 = U(1:p, data.MVIndex(4));

    % -------------------- Extract required states over steps 2..p+1 --------------------
    % X is usually (p+1)-by-Nx.
    % Using rows 2..p+1 corresponds to predicted states after applying controls at 1..p.
    X1  = X(2:p+1, 1);
    X2  = X(2:p+1, 2);
    X3  = X(2:p+1, 3);   
    X4  = X(2:p+1, 4);
    X5  = X(2:p+1, 5);
    X6  = X(2:p+1, 6);
    X7  = X(2:p+1, 7);
    X8  = X(2:p+1, 8);
    X9  = X(2:p+1, 9);
    X10 = X(2:p+1, 10);
    X11 = X(2:p+1, 11);
    X12 = X(2:p+1, 12);

    % -------------------- Extract MDs used for U4 bounds --------------------
    % V    = Speed (MD3) for steps 1..p
    % rho  = Air density (MD4) for steps 1..p
    V   = U(1:p, data.MDIndex(3));
    rho = U(1:p, data.MDIndex(4));

    % -------------------- Compute dynamic bounds on U4 --------------------
    % Lower bound: proportional to rho * V * A_c
    U4_Lb = rho .* V * A_c;

    % Upper bound: same baseline + fan margin
    U4_Ub = rho .* V * A_c + dot_m_fan;

    % -------------------- Build inequality constraints (must satisfy <= 0) --------------------
    %
    % For a bound of the form:
    %    U4 >= U4_Lb
    % rewrite as:
    %    U4_Lb - U4 <= 0
    %
    % For:
    %    U4 <= U4_Ub
    % rewrite as:
    %    U4 - U4_Ub <= 0
    %
    % For soft state bounds:
    %    Xi >= (Lower - e)  ->  (Lower - e) - Xi <= 0
    %    Xi <= (Upper + e)  ->  Xi - (Upper + e) <= 0
    %
    % Note: e is a single scalar slack used to relax all these bounds together.
    cineq = [ ...
        % --- MV4 (dot_m_air) bounds ---
        U4_Lb - U4;
        U4 - U4_Ub;

        % --- State 1 bounds (soft) ---
        (10 - e) - X1;
        X1 - (50 + e);

        % --- State 2 bounds (soft) ---
        (10 - e) - X2;
        X2 - (70 + e);

        % --- State 3 (SoC)  commented out---
        % 0.1 - X3;
        % X3 - 1;

        % --- State 4 bounds (soft) ---
        (10 - e) - X4;
        X4 - (90 + e);

        % --- State 5 bounds (soft) ---
        (10 - e) - X5;
        X5 - (80 + e);

        % --- State 6 bounds (soft) ---
        (10 - e) - X6;
        X6 - (90 + e);

        % --- State 7 bounds (soft) ---
        (10 - e) - X7;
        X7 - (80 + e);

        % --- State 8 bounds (soft) ---
        (10 - e) - X8;
        X8 - (80 + e);

        % --- State 9 bounds (soft) ---
        (10 - e) - X9;
        X9 - (90 + e);

        % --- State 10 bounds (soft) ---
        (10 - e) - X10;
        X10 - (80 + e);

        % --- State 11 bounds (soft) ---
        (10 - e) - X11;
        X11 - (80 + e);

        % --- State 12 bounds (soft) ---
        (10 - e) - X12;
        X12 - (90 + e);
    ];

end