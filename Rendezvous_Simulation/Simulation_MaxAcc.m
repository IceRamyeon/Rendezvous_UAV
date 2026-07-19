function [sim_results, sim_out] = Simulation_MaxAcc(cfg, theory)
%% Simulation_MaxAcc.m
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
    case 'RDPG_ACC'
        % 초기 리드각 계산
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_ACC(cfg.gain_k, cfg.limit_acc, cfg.r_allow, dt, init_sigma);
        
    case 'DPG'
        % 기존 DPG 객체 생성
        dpg_target_deg = cfg.target_lead_angle_deg;
        missile_guidance = DPG(dpg_target_deg, cfg.gain_k, cfg.limit_acc);
        
    case 'RDPG'
        % RDPG 객체 생성
        init_sigma = current_psi_p - lambda_init;
        rate_limit_rad = 15 * (pi/180); 
        test_mode_flag = 0;
        missile_guidance = RDPG(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                rate_limit_rad, dt, init_sigma, test_mode_flag);
                                
    case 'RDPG_VT'
        % [추가된 부분] RDPG_VT 객체 생성
        init_sigma_real = current_psi_p - lambda_init;
        init_sigma_vt = 0; % 가상 타겟용 초기 시선각 오차는 0으로 시작
        
        % 가상 타겟 전용 유도 파라미터 (필요하면 main에서 cfg로 넘기도록 빼도 돼)
        test_mode_flag_vt = 0;
        
        missile_guidance = RDPG_VT(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                   dt, init_sigma_real, ...
                                   cfg.r_allow, test_mode_flag_vt, init_sigma_vt, ...
                                   cfg.r_vt, cfg.theta_vt_rad);

    case 'RDPG_MIN'
        % 초기 리드각 계산
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_MIN(cfg.gain_k, cfg.limit_acc, cfg.a_min, cfg.r_allow, dt, init_sigma);
        
    case 'RDPG_r_f'
        % 초기 리드각 계산
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_r_f(cfg.gain_k, cfg.limit_acc, cfg.r_f_escape, cfg.r_allow, dt, init_sigma); 
    otherwise
        error('[오류] 알 수 없는 GUIDANCE_MODE.');
end

num_steps = length(time);
hist_state = zeros(17, num_steps); 
    
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
    
    lambda_dot_vt = NaN;
    x_vt_log = NaN;
    y_vt_log = NaN;
    r_f_log = NaN;
    
    switch cfg.GUIDANCE_MODE
        case 'RDPG_ACC'
            [acc_cmd, sigma_ref_new, mode_flag] = missile_guidance.compute_command(V_p, lambda_dot, sigma_p, r, sigma_t);
        case 'DPG'
            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
            sigma_ref_new = sigma_p; 
            mode_flag = -1;
        case 'RDPG'
            [acc_cmd, sigma_ref_new] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
            mode_flag = -1;
        case 'RDPG_VT'
            % 1. 가상 타겟(Virtual Target)의 전역 좌표 계산
            gamma_vt = current_psi_t - cfg.theta_vt_rad; 
            X_vt = current_Xt + cfg.r_vt * cos(gamma_vt);
            Y_vt = current_Yt + cfg.r_vt * sin(gamma_vt);

            % 2. Pursuer와 가상 타겟 사이의 상대 기구학 계산
            Rx_vt = X_vt - current_Xp;
            Ry_vt = Y_vt - current_Yp;
            r_vt_dist = sqrt(Rx_vt^2 + Ry_vt^2);
            lambda_vt = atan2(Ry_vt, Rx_vt);

            % 3. 가상 타겟 기준 시선각 오차 
            sigma_p_vt = current_psi_p - lambda_vt;
            sigma_t_vt = current_psi_t - lambda_vt;

            % 4. 가상 타겟 기준 시선각 변화율 (lambda_dot_vt) 계산
            lambda_dot_vt = (V_t * sin(sigma_t_vt) - V_p * sin(sigma_p_vt)) / r_vt_dist;

            % 배열에 로깅하기 위해 변수 할당
            x_vt_log = X_vt;
            y_vt_log = Y_vt;

            [acc_cmd, sigma_ref_new, mode_flag] = missile_guidance.compute_command(...
            V_p, r, lambda_dot, sigma_p, sigma_t, ...
            r_vt_dist, lambda_dot_vt, sigma_p_vt, sigma_t_vt);
        
        case 'RDPG_MIN'
            [acc_cmd, sigma_ref_new, mode_flag, r_f_log] = missile_guidance.compute_command(V_p, lambda_dot, sigma_p, r, sigma_t);

        case 'RDPG_r_f'
            [acc_cmd, sigma_ref_new, mode_flag] = missile_guidance.compute_command(V_p, lambda_dot, sigma_p, r, sigma_t);
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
    
    heading_diff_deg = abs(wrapTo180((current_psi_p - current_psi_t) * R2D));
    if r <= r_allow && heading_diff_deg < th_psi_deg
        if cfg.stop_condition == 1
            hist_state = hist_state(:, 1:i);
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
sim_out.init.Xp_i = Xp_i; sim_out.init.Yp_i = Yp_i;
sim_out.init.Xt_i = Xt_i; sim_out.init.Yt_i = Yt_i;
sim_out.init.psi_p = psi_p; sim_out.init.psi_t = psi_t;
sim_out.init.lambda_init = lambda_init;
sim_out.init.r_i = r_i; sim_out.init.bearing_deg = cfg.theta_pursuer;
sim_out.param.V_p = V_p; sim_out.param.V_t = V_t;
sim_out.param.At_constant = cfg.At_constant;

%% 플롯 함수 호출
fprintf('\n>>> 결과 시각화 모듈 실행... <<<\n');
Animate_Trajectory(cfg, sim_out);
Plot_Static_Results(cfg, sim_out);
Plot_MaxAccPoint(cfg, sim_out, theory); 

end