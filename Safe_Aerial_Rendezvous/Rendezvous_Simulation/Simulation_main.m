function [sim_results, sim_out] = Simulation_main(cfg, theory)
%% Simulation_main.m
R2D = 180/pi;
D2R = pi/180;

dt = cfg.dt_simul;
tf = cfg.tf;
time = 0:dt:tf;

V_p = cfg.V_p; V_t = cfg.V_t;
r_allow = cfg.r_allow;
th_psi_deg = cfg.th_psi_deg;

%% 초기 상태 정의
Xt_i = cfg.Xt_input_km * 1000; Yt_i = cfg.Yt_input_km * 1000;
psi_t = cfg.psi_ti_deg * D2R;
r_i = cfg.r_pursuer;
bearing = cfg.theta_pursuer * D2R;

rotation_diff_deg = cfg.psi_ti_deg - 90.0;
psi_p_deg = cfg.psi_p_auto + rotation_diff_deg;
psi_p = psi_p_deg * D2R;

global_los_rad = psi_t - bearing; 
Xp_i = Xt_i + r_i * cos(global_los_rad);
Yp_i = Yt_i + r_i * sin(global_los_rad);

current_Xp = Xp_i; current_Yp = Yp_i;
current_Xt = Xt_i; current_Yt = Yt_i;
current_psi_p = psi_p; current_psi_t = psi_t;
lambda_init = atan2(Yt_i - Yp_i, Xt_i - Xp_i);

switch cfg.GUIDANCE_MODE
    case 'DPG'
        % 기존 DPG 객체 생성
        missile_guidance = DPG(cfg.sigma_p0_deg, cfg.gain_k, cfg.limit_acc);

    case 'PPNG'
        missile_guidance = PPNG(cfg.N_PPNG, cfg.Bias_PPNG, cfg.limit_acc);

    case 'VDPG'
        missile_guidance = VDPG(cfg.gain_k, cfg.limit_acc);

    case 'RDPG'
        % RDPG 객체 생성
        init_sigma = current_psi_p - lambda_init;
        rate_limit_rad = 15 * (pi/180); 
        test_mode_flag = 0;
        missile_guidance = RDPG(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                rate_limit_rad, dt, init_sigma, test_mode_flag);

    case 'RDPG_ACC'
        % 초기 리드각 계산
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_ACC(cfg.gain_k, cfg.limit_acc, cfg.r_allow, dt, init_sigma);

    case 'RDPG_MIN'
        init_sigma = current_psi_p - lambda_init;
        rate_limit_rad = cfg.rate_limit_deg;
        missile_guidance = RDPG_MIN(cfg.gain_k, cfg.limit_acc, cfg.a_min, cfg.r_allow, dt, init_sigma, rate_limit_rad);

    case 'RDPG_FRS'
        init_sigma = current_psi_p - lambda_init;
        sigma_pref_rad = deg2rad(cfg.sigma_pref_deg);
        rate_limit_rad = deg2rad(cfg.rate_limit_deg);
        missile_guidance = RDPG_FRS(cfg.gain_k, cfg.limit_acc, cfg.r_allow, dt, init_sigma, ...
        sigma_pref_rad, cfg.t_for, cfg.sigma_FRS_list, rate_limit_rad); 
    otherwise
        error('[오류] 알 수 없는 GUIDANCE_MODE.');
end

num_steps = length(time);
hist_state = zeros(17, num_steps); 

% RDPG_FRS 용 cell 초기화
point_value_hist = cell(1, num_steps);

% VDPG 용 초기 r0 저장
r_0 = cfg.r_pursuer;
    
% 시뮬레이션 연산[cite: 4]
for i = 1:num_steps
    Rx = current_Xt - current_Xp; Ry = current_Yt - current_Yp;
    r = sqrt(Rx^2 + Ry^2);
    lambda = atan2(Ry, Rx);
    
    sigma_p = current_psi_p - lambda;
    sigma_t = current_psi_t - lambda;
    
    r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
    V_c = -r_dot;
    lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
    
    % lambda_dot_vt = NaN;
    x_vt_log = NaN;
    y_vt_log = NaN;
    r_f_log = NaN;
    
    current_point_value = [];

    switch cfg.GUIDANCE_MODE
        case 'DPG'
            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
            sigma_ref_new = sigma_p; 
            mode_flag = -1;
            
        case 'PPNG'
            acc_cmd = missile_guidance.compute_commandPPNG(V_p, lambda_dot);
            sigma_ref_new = 0;
            mode_flag = -1;

        case 'VDPG'
            acc_cmd = missile_guidance.compute_commandVDPG(V_p, lambda_dot, sigma_p, r, r_0, cfg.sigma_p0_deg);
            sigma_ref_new = 0;
            mode_flag = -1;

        case 'RDPG'
            [acc_cmd, sigma_ref_new] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
            mode_flag = -1;

        case 'RDPG_ACC'
            [acc_cmd, sigma_ref_new, mode_flag] = missile_guidance.compute_command(V_p, lambda_dot, sigma_p, r, sigma_t);
        
        case 'RDPG_MIN'
            [acc_cmd, sigma_ref_new, mode_flag, r_f_log] = missile_guidance.compute_command(V_p, lambda_dot, sigma_p, r, sigma_t);

        case 'RDPG_FRS'
            rel_x = current_Xp - current_Xt;
            rel_y = current_Yp - current_Yt;
            [acc_cmd, sigma_ref_new, mode_flag, current_point_value] = missile_guidance.compute_command(...
            V_p, lambda_dot, sigma_p, r, sigma_t, V_t, current_psi_t, rel_x, rel_y);

        otherwise
            error('으헤.. 알 수 없는 GUIDANCE_MODE야, 선생.');
    end
    
    omega_p = acc_cmd / V_p;
    current_psi_p = current_psi_p + omega_p * dt;
    current_Xp = current_Xp + V_p * cos(current_psi_p) * dt;
    current_Yp = current_Yp + V_p * sin(current_psi_p) * dt;
    
    current_Xt = current_Xt + V_t * cos(current_psi_t) * dt;
    current_Yt = current_Yt + V_t * sin(current_psi_t) * dt;
    
    % 상태 저장
    hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; ...
    sigma_p; sigma_t; V_c; acc_cmd; sigma_ref_new; mode_flag; current_Xp; current_Yp; ...
    current_Xt; current_Yt; x_vt_log; y_vt_log; ...
    r_f_log];
    point_value_hist{i} = current_point_value;
    
    heading_diff_deg = abs(wrapTo180((current_psi_p - current_psi_t) * R2D));
    if r <= r_allow && heading_diff_deg < th_psi_deg
        if cfg.stop_condition == 1
            hist_state = hist_state(:, 1:i);
            point_value_hist = point_value_hist(1:i);
            time = time(1:i);
            break;
        end
    end
end

%% 데이터 패키징 및 실측값 추출
r_data = hist_state(1, :);
sigma_t_data = hist_state(6, :) * R2D;
acc_data = hist_state(8, :);

[~, max_idx] = max(abs(acc_data));
r_sim_at_max = r_data(max_idx);
sigma_t_sim_at_max = sigma_t_data(max_idx);
a_sim_at_max = acc_data(max_idx);

% --- main_MaxAcc로 던져줄 결과 구조체 채우기 ---
sim_results.r_sim_at_max = r_sim_at_max;
sim_results.sigma_t_sim_at_max = sigma_t_sim_at_max;
sim_results.a_sim_at_max = a_sim_at_max;

% 외부 플롯 호환용 데이터 팩
sim_out.time = time;
sim_out.hist_state = hist_state;
sim_out.point_value_hist = point_value_hist;
sim_out.init.Xp_i = Xp_i; sim_out.init.Yp_i = Yp_i;
sim_out.init.Xt_i = Xt_i; sim_out.init.Yt_i = Yt_i;
sim_out.init.psi_p = psi_p; sim_out.init.psi_t = psi_t;
sim_out.init.lambda_init = lambda_init;
sim_out.init.r_i = r_i; sim_out.init.bearing_deg = cfg.theta_pursuer;
sim_out.param.V_p = V_p; sim_out.param.V_t = V_t;
sim_out.param.At_constant = cfg.At_constant;

%% 플롯 함수 호출
fprintf('\n>>> 결과 시각화 모듈 실행... <<<\n');
Animation(cfg, sim_out);
Plot_Static_Results(cfg, sim_out);
Plot_MaxAccPoint(cfg, sim_out, theory); 

end