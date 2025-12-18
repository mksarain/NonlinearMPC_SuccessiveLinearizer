function ceq = aircraftEqCon(X, U, data)
% aircraftEqCon
% Custom equality constraint function for nlmpc.
%
% Purpose:
%   Enforces power-balance (or energy-flow balance) equality at each
%   prediction step over the horizon:
%
%     eta_g*eta_r*P_gen  +  eta_c*P_batt  =  P_prop / (eta_bus*eta_m*eta_i)
%
% Inputs (nlmpc convention):
%   X    : predicted state trajectory (not used here, but required by signature)
%   U    : predicted input trajectory (contains both MVs and MDs)
%   data : struct containing horizon length and column indices for MV/MD
%
% Output:
%   ceq  : vector of equality constraints (length p). The optimizer forces
%          ceq == 0 (within tolerance) at the solution.

    % -------------------- Efficiencies (constants) --------------------
    eta_c   = 0.93;
    eta_g   = 0.9;
    eta_r   = 0.93;
    eta_bus = 0.99;
    eta_m   = 0.9;
    eta_i   = 0.93;

    % -------------------- Horizon length --------------------
    % p = prediction horizon steps
    p = data.PredictionHorizon;

    % -------------------- Extract relevant trajectories from U --------------------
    % u1: MV1 trajectory (P_batt) for steps 1..p
    u1 = U(1:p, data.MVIndex(1));

    % u2: MV2 trajectory (P_gen) for steps 1..p
    u2 = U(1:p, data.MVIndex(2));

    % md: MD2 trajectory (P_prop) for steps 1..p
    md = U(1:p, data.MDIndex(2));

    % -------------------- Equality constraint definition --------------------
    % This returns a p-by-1 vector. nlmpc enforces ceq(k) = 0 for all k.
    %
    % Interpretation per step k:
    %   (available electrical power from generator after efficiencies)
    % + (battery power after converter)
    % - (required propulsion power mapped through bus*motor*inverter)
    % = 0
    ceq = eta_g * eta_r * u2 + eta_c * u1 - (md/(eta_bus * eta_m * eta_i));
end