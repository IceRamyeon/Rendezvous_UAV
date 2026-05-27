function sim_results = Simulation_MaxAcc(cfg, theory)
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
r_i = cfg.r_from_region_m;
bearing = cfg.bearing_from_region * D2R;

rotation_diff_deg = cfg.psi_ti_deg - 90.0;
psi_p_deg = cfg.psi_p_from_region + rotation_diff_deg;
psi_p = psi_p_deg * D2R;

global_los_rad = psi_t - bearing; 
Xp_i = Xt_i + r_i * cos(global_los_rad);
Yp_i = Yt_i + r_i * sin(global_los_rad);

current_Xp = Xp_i; current_Yp = Yp_i;
current_Xt = Xt_i; current_Yt = Yt_i;
current_psi_p = psi_p; current_psi_t = psi_t;
lambda_init = atan2(Yt_i - Yp_i, Xt_i - Xp_i);

% DPG 객체 생성 (G-Limit 중복연산 방지)
dpg_target_deg = cfg.target_lead_angle_deg;
missile_guidance = DPG(dpg_target_deg, cfg.gain_k, cfg.limit_acc); 

num_steps = length(time);
hist_state = zeros(8, num_steps); 

% 시뮬레이션 연산
for i = 1:num_steps
    Rx = current_Xt - current_Xp; Ry = current_Yt - current_Yp;
    r = sqrt(Rx^2 + Ry^2);
    lambda = atan2(Ry, Rx);
    
    sigma_p = current_psi_p - lambda;
    sigma_t = current_psi_t - lambda;
    
    r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
    V_c = -r_dot;
    lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
    
    acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
    
    omega_p = acc_cmd / V_p;
    current_psi_p = current_psi_p + omega_p * dt;
    current_Xp = current_Xp + V_p * cos(current_psi_p) * dt;
    current_Yp = current_Yp + V_p * sin(current_psi_p) * dt;
    
    current_Xt = current_Xt + V_t * cos(current_psi_t) * dt;
    current_Yt = current_Yt + V_t * sin(current_psi_t) * dt;
    
    hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd];
    
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
sim_out.init.r_i = r_i; sim_out.init.bearing_deg = cfg.bearing_from_region;
sim_out.param.V_p = V_p; sim_out.param.V_t = V_t;
sim_out.param.At_constant = cfg.At_constant;

%% 플롯 함수 호출
fprintf('\n>>> 결과 시각화 모듈 실행... <<<\n');
Animate_Trajectory(cfg, sim_out);
Plot_Static_Results(cfg, sim_out);
Plot_MaxAccPoint(cfg, sim_out, theory); 

end