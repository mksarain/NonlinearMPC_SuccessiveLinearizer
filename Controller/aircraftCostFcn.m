%% Custom stage cost function for Model Predictive Control (MPC)
% The same cost function used in the thesis excluding the masses-of-components term
% and the slack-variable term (slack term is g*e).
%
% This function is called by nlmpc at every optimizer iteration to evaluate the
% economic objective over the prediction horizon.
%
% Signature meaning (nlmpc convention):
%   X    : predicted state trajectory over horizon (typically (p+1)-by-Nx)
%   U    : predicted input trajectory over horizon (typically (p+1)-by-Nu)
%   e    : slack variable used for soft constraints (scalar, >=0)
%   data : structure providing indices and horizon info (e.g., MVIndex, PredictionHorizon)
%
% Output:
%   obj  : scalar objective value to be minimized

function obj = aircraftCostFcn(X, U, e, data)

    % Initialize the objective accumulator to zero.
    % The solver will minimize this scalar.
    obj = 0;

    % Extract only the manipulated variables (MVs) from the full input matrix U.
    % data.MVIndex lists which columns of U correspond to MVs.
    % Here we keep the MVs (P_batt, P_gen, m_dot, m_dot_air).
    u = U(:,data.MVIndex(1:4));

    % Extract the battery temperature state trajectory from X.
    % X(:,1) means: first state = Tw_batt (as defined in nlobj.States).
    T_batt = X(:,1);

    % Create 4x4 weighting matrices for penalizing:
    %   R: magnitude of MVs (Uk)
    %   Q: rate-of-change of MVs (dUk)
    % They are initialized to zeros, then selected diagonal terms are set.
    d.p.R = zeros(4,4);
    d.p.Q = zeros(4,4);

    % Penalize the 3rd and 4th MV magnitudes (m_dot and m_dot_air).
    % (No penalty on MV1 and MV2 magnitude here.)
    d.p.R(3,3) = 1;
    d.p.R(4,4) = 1;

    % Penalize the changes (move-suppression) of the 3rd and 4th MV.
    % Larger weights => smoother changes (more reluctance to move).
    d.p.Q(3,3) = 1e2;
    d.p.Q(4,4) = 1e3;

    % Extract the generator power MV trajectory (2nd MV).
    % This is used for tracking an economic reference.
    P_gen = U(:, data.MVIndex(2));

    % Generator power reference (economic target).
    P_ref = 300e3;

    % Weight for generator power tracking term (small => gentle economic push).
    alpha = 1e-4;

    % Weight for battery temperature tracking term.
    beta = 1e1;

    % Battery temperature reference (desired operating temperature).
    T_ref = 20;

    % Slack penalty weight:
    % e>0 means constraint violation.
    % Large g strongly discourages violating constraints.
    g = 1e4;

    % Sum stage costs across the prediction horizon.
    % data.PredictionHorizon = p
    %
    % Note: This loop uses u(k+1,:) and T_batt(k+1,:), so X and U must have
    % at least (p+1) rows, which is typical in nlmpc (k=1..p, states at 1..p+1).
    for k = 1:data.PredictionHorizon

        % Uk = MV vector at step k (1x4).
        Uk = u(k, :);

        % dUk = MV increment from step k to k+1 (1x4).
        % This penalizes how aggressively the MVs move.
        dUk = u(k+1,:) - u(k,:);

        % MV magnitude cost: Uk * R * Uk'
        % With diagonal R, this is a weighted sum of squares of selected MVs.
        Ji = Uk * d.p.R * Uk';

        % MV move cost: dUk * Q * dUk'
        % Penalizes rate-of-change of selected MVs.
        Ju = dUk * d.p.Q * dUk';

        % Generator power tracking cost:
        % Encourages P_gen(k) to be close to P_ref.
        Jp = alpha * (P_gen(k,:)-P_ref).^2;

        % Battery temperature tracking cost:
        % Uses T_batt(k+1) because the state at k+1 is the result of applying Uk at k.
        Jt = beta * (T_batt(k+1,:) - T_ref).^2;

        % Add all stage costs plus slack penalty.
        % Slack term is added each stage here; this effectively scales the slack
        % penalty by horizon length.
        obj = obj + Ji + Ju + Jp + Jt + g * e;

        % Alternative (commented) version without temperature term:
        % obj = obj + Ji + Ju + Jp + g * e;
    end
end