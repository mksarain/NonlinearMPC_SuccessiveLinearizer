function [Temp, Speed, rho] = parameters(t)
    %% Calculates aircraft altitude in ft and m

    T_off = 60;
    t_cruise = 420;
    t_landing = 3660;
    t_final = 4440;

    r_slope = (4000-50)/(t_cruise-T_off);
    r_slope2 =(0-4000)/(t_final-t_landing);
    
    step = 50;
    
    r_ramp1 = zeros(size(t));
    r_ramp1(t >= T_off) = r_slope * (t(t >= T_off) - T_off);
    
    r_ramp2 = zeros(size(t));
    r_ramp2(t >= t_cruise) = -r_slope * (t(t >= t_cruise) - t_cruise);
    
    r_ramp3 = zeros(size(t));
    r_ramp3(t >= t_landing) = r_slope2 * (t(t >= t_landing) - t_landing);
    
    h = step + r_ramp1 + r_ramp2 + r_ramp3;
    h(h < 0) = 0;
    
    h = 0.3048 * h;   % convert from feet to meters

    %% Calculates the ambient temperature and speed of sound (Obtained from Bing AI)
    Gamma = 1.4; % air specific heat ratio
    R = 287.053; % J/(kg.K) % gas constant
    T_amp = -6.5 * 1e-3 * h + 15;  % ambient temperature
    a = sqrt(Gamma * R * (T_amp + 273.15)); % speed of sound

    %% Calculates aircraft speed in knots; converted to Mach and m/s

    s_slope_1 = 77/60; %0.25 / 1
    s_slope_2 = (125 - 77) / (t_cruise - T_off);
    s_slope_3 = (78 - 125) / (t_final - t_landing);
    
    S_ramp = zeros(size(t));
    S_ramp(t <= T_off) = s_slope_1 * t(t <= T_off);
    
    S_ramp1 = zeros(size(t));
    S_ramp1(t > T_off & t < t_cruise) = s_slope_2 * (t(t > T_off & t < t_cruise) - T_off) + 77;
    
    S_ramp2 = zeros(size(t));
    S_ramp2(t >= t_cruise & t <= t_landing) = 125;
    
    S_ramp3 = zeros(size(t));
    S_ramp3(t > t_landing & t <= t_final) = s_slope_3 * (t(t > t_landing & t <= t_final) - t_landing) + 125;
    
    S_ramp4 = zeros(size(t));
    S_ramp4(t > t_final) = s_slope_3 * (t(t > t_final) - t_final) + 78;
    
    
    knot = S_ramp + S_ramp1 + S_ramp2 + S_ramp3 + S_ramp4; % True aircraft speed (TSA) in knots
    
    knot = 0.5144 * knot; %  TSA in m/s 
    Mach = knot / a; % Mach number

    Speed = Mach * 343; % Convert Mach to m/s
    %% Calculates the ram temperature
    Temp = T_amp * (1+Mach^2 * ((Gamma-1)/2));

    %% Calculate Pressure altitude and air density

    P0 = 101325; % Pressure at standard see level in Pascal
    P = P0 * (1 - 2.25577e-5 * h).^5.25588; % Pressure altitude % Obtained from Bing AI

    rho = P/(R* (T_amp+273.15));
end