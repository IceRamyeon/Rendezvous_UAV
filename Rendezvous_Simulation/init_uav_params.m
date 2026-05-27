function UAV = init_uav_params()
    % Aerosonde UAV Physical & Aerodynamic Parameters
    % Beased on "Small Unmanned Aircraft: Theory and Practice" by Randal W. Beard and Timothy W. McLain, 2012.
    % All Data are referred to Appendix E; Table E.1, E.2

    %% Table E.1: Physical Parameters
    UAV.m    = 11.0;       % [kg] Mass
    UAV.Jx   = 0.824;      % [kg-m^2]
    UAV.Jy   = 1.135;      % [kg-m^2]
    UAV.Jz   = 1.759;      % [kg-m^2]
    UAV.Jxz  = 0.120;      % [kg-m^2]
    UAV.S    = 0.55;       % [m^2] Wing area
    UAV.b    = 2.9;        % [m] Wingspan
    UAV.c    = 0.19;       % [m] Chord
    UAV.rho  = 1.268;      % [kg/m^3] Air density
    UAV.e    = 0.9;        % Oswald efficiency factor

    %% Table E.1: Motor Parameters
    UAV.V_max   = 44.4;     % [V] Max voltage
    UAV.D_prop  = 0.508;    % [m] Propeller diameter
    UAV.K_V     = 0.0659;   % [V-s/rad]
    UAV.K_Q     = 0.0659;   % [N-m]
    UAV.R_motor = 0.042;    % [Ohm] Motor resistance
    UAV.i0      = 1.5;      % [A] No-load current
    
    % Propeller torque/thrust coefficients
    UAV.C_Q2    = -0.01664;
    UAV.C_Q1    = 0.004970;
    UAV.C_Q0    = 0.005230;
    UAV.C_T2    = -0.1079;
    UAV.C_T1    = -0.06044;
    UAV.C_T0    = 0.09357;

    %% Table E.2: Longitudinal Aerodynamic Coefficients
    UAV.C_L_0      = 0.23;
    UAV.C_D_0      = 0.043;
    UAV.C_m_0      = 0.0135;
    UAV.C_L_alpha  = 5.61;
    UAV.C_D_alpha  = 0.030;
    UAV.C_m_alpha  = -2.74;
    UAV.C_L_q      = 7.95;
    UAV.C_D_q      = 0;
    UAV.C_m_q      = -38.21;
    UAV.C_L_delta_e = 0.13;
    UAV.C_D_delta_e = 0.0135;
    UAV.C_m_delta_e = -0.99;
    
    % Blending & High Alpha Parameters
    UAV.M          = 50;
    UAV.alpha0     = 0.47;
    UAV.epsilon    = 0.16;
    UAV.C_D_p      = 0;

    %% Table E.2: Lateral Aerodynamic Coefficients
    UAV.C_Y_0      = 0;
    UAV.C_l_0      = 0;
    UAV.C_n_0      = 0;
    UAV.C_Y_beta   = -0.83;
    UAV.C_l_beta   = -0.13;
    UAV.C_n_beta   = 0.073;
    UAV.C_Y_p      = 0;
    UAV.C_l_p      = -0.51;
    UAV.C_n_p      = -0.069;
    UAV.C_Y_r      = 0;
    UAV.C_l_r      = 0.25;
    UAV.C_n_r      = -0.095;
    UAV.C_Y_delta_a = 0.075;
    UAV.C_l_delta_a = 0.17;
    UAV.C_n_delta_a = -0.011;
    UAV.C_Y_delta_r = 0.19;
    UAV.C_l_delta_r = 0.0024;
    UAV.C_n_delta_r = -0.069;

end