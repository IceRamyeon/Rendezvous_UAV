function Randezvous_Simulation(cfg)

%% 1. 파라미터 및 초기 설정
R2D = 180/pi;
D2R = pi/180;

GUIDANCE_MODE = cfg.GUIDANCE_MODE; 
dt = cfg.dt_simul;
tf = cfg.tf;
time = 0:dt:tf;

V_p = cfg.V_p;
V_t = cfg.V_t;
At_constant = cfg.At_constant;
At_cmd = cfg.At_cmd;
At_current = 0;

r_allow = cfg.r_allow;
th_psi_deg = cfg.th_psi_deg;

%% 2. 초기 상태 정의
Xt_i = cfg.Xt_input_km * 1000;
Yt_i = cfg.Yt_input_km * 1000;
psi_t = cfg.psi_ti_deg * D2R;

r_i = cfg.r_from_region_m;
bearing_deg = cfg.bearing_from_region;
bearing = bearing_deg * D2R;

rotation_diff_deg = cfg.psi_ti_deg - 90.0;
psi_p_deg = cfg.psi_p_from_region + rotation_diff_deg;
psi_p = psi_p_deg * D2R;

global_los_rad = psi_t - bearing; 

Xp_i = Xt_i + r_i * cos(global_los_rad);
Yp_i = Yt_i + r_i * sin(global_los_rad);

current_Xp = Xp_i; current_Yp = Yp_i;
current_Xt = Xt_i; current_Yt = Yt_i;
current_psi_p = psi_p;
current_psi_t = psi_t;

dx = Xt_i - Xp_i;
dy = Yt_i - Yp_i;
lambda_init = atan2(dy, dx);

% --- Guidance Object 생성 ---
switch GUIDANCE_MODE
    case 'PPNG'
        missile_guidance = PPNG(cfg.N, cfg.bias_acc, cfg.limit_acc); 
    case 'DPG'
        if isfield(cfg, 'target_lead_angle_deg')
            dpg_target_deg = cfg.target_lead_angle_deg;
            use_auto_angle = false;
        else
            dpg_target_deg = 0;
            use_auto_angle = true;
        end
        missile_guidance = DPG(dpg_target_deg, cfg.gain_k, cfg.limit_acc); 
        if exist('use_auto_angle', 'var') && use_auto_angle
            actual_sigma_init_rad = current_psi_p - lambda_init;
            missile_guidance.sigma_des = actual_sigma_init_rad;
        end
    case 'VDPG'
        missile_guidance = VDPG(cfg.gain_k, cfg.limit_acc);
    case 'RDPG'
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                cfg.rate_limit, dt, init_sigma, cfg.alpha, cfg.RDPG_test);
    case 'RDPG_T'
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_T(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                  cfg.rate_limit, dt, init_sigma, ...
                                  cfg.RDPG_T_MODE, cfg.alpha, cfg.sigma_offset, cfg.RDPG_test);
    case 'RDPG_SAFE'
        init_sigma = current_psi_p - lambda_init;
        missile_guidance = RDPG_SAFE(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                    cfg.rate_limit, dt, init_sigma, V_p, V_t);
    otherwise
        error('모드 없음');
end

fprintf('>>> 변환 및 초기화 완료 <<<\n');
fprintf('Target  Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xt_i, Yt_i, cfg.psi_ti_deg);
fprintf('Pursuer Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xp_i, Yp_i, psi_p_deg);

%% 3. 메인 시뮬레이션 루프 (계산만 수행)
num_steps = length(time);
if strcmp(GUIDANCE_MODE, 'RDPG')
    hist_state = zeros(9, num_steps);
elseif strcmp(GUIDANCE_MODE, 'RDPG_T')
    hist_state = zeros(11, num_steps);
elseif strcmp(GUIDANCE_MODE, 'RDPG_SAFE')
    hist_state = zeros(12, num_steps);
else
    hist_state = zeros(8, num_steps); 
end

fprintf('>>> 시뮬레이션 연산 진행 중... <<<\n');

r0 = r_i;                         
sigma_init = psi_p - lambda_init; 
is_stop = 0;
final_msg = '';

for i = 1:num_steps
    
    % --- [A] Navigation ---
    Rx = current_Xt - current_Xp;
    Ry = current_Yt - current_Yp;
    r = sqrt(Rx^2 + Ry^2);
    lambda = atan2(Ry, Rx);
    
    sigma_p = current_psi_p - lambda;
    sigma_t = current_psi_t - lambda;
    
    r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
    V_c = -r_dot;
    lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
    
    % --- [B] Guidance Command ---
    switch GUIDANCE_MODE
        case 'PPNG'
            acc_cmd = missile_guidance.compute_commandPPNG(V_c, lambda_dot);
            At_current = At_constant;
        case 'DPG'
            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
            At_current = At_constant;
        case 'VDPG'
            acc_cmd = missile_guidance.compute_commandVDPG(V_p, lambda_dot, sigma_p, r, r0, sigma_init);
            At_current = At_constant;
        case 'RDPG'
            [acc_cmd, current_sigma_ref] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
            At_current = At_constant;
        case 'RDPG_T'
            [acc_cmd, current_sigma_ref, tanker_turn_on] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop);
            if tanker_turn_on ~= 0
                At_current = At_cmd * tanker_turn_on; 
            else
                At_current = 0;
            end
        case 'RDPG_SAFE'
            X_state = [r; sigma_t; sigma_p];
            [acc_cmd, current_sigma_ref, V_clf, H_cbf, slack_opt] = missile_guidance.compute_commandRDPG_SAFE(X_state, lambda_dot);
            At_current = At_constant; 
    end
    
    % --- [C] Kinematics Update ---
    omega_p = acc_cmd / V_p;
    current_psi_p = current_psi_p + omega_p * dt;
    current_Xp = current_Xp + V_p * cos(current_psi_p) * dt;
    current_Yp = current_Yp + V_p * sin(current_psi_p) * dt;
    
    omega_t = At_current / V_t;
    current_psi_t = current_psi_t + omega_t * dt;
    current_Xt = current_Xt + V_t * cos(current_psi_t) * dt;
    current_Yt = current_Yt + V_t * sin(current_psi_t) * dt;
    
    % Data Logging
    if strcmp(GUIDANCE_MODE, 'RDPG')
        hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd; current_sigma_ref];
    elseif strcmp(GUIDANCE_MODE, 'RDPG_T')
        hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd; current_sigma_ref; is_stop; At_current];
    elseif strcmp(GUIDANCE_MODE, 'RDPG_SAFE')
        hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd; current_sigma_ref; V_clf; H_cbf; slack_opt];
    else
        hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd];
    end
    
    % --- [D] Stop Condition ---
    stop_condition = cfg.stop_condition;
    heading_diff_deg = abs(wrapTo180((current_psi_p - current_psi_t) * R2D));

    if ~is_stop
        is_reached = false;
        if V_c < 0 && heading_diff_deg < th_psi_deg && r < r_allow
            final_msg = '랑데부 성공 (CPA 감지, V_c < 0)';
            is_reached = true;
        elseif r < r_allow && heading_diff_deg < th_psi_deg
            final_msg = '랑데부 성공 (r < r_allow)';
            is_reached = true;
        elseif i > 1 && r > hist_state(1, i-1) && r < r_allow && heading_diff_deg < th_psi_deg
            final_msg = sprintf('랑데뷰 성공 (최소거리=%.3fm)', hist_state(1, i));
            is_reached = true;
        end

        if is_reached
            is_stop = 1; 
            if stop_condition
                hist_state = hist_state(:, 1:i); 
                time = time(1:i);
                break; 
            end
        end
    end

    if i == num_steps && ~is_stop
        final_msg = sprintf('Time Limit Reached (t=%.1fs, r=%.1fm)', time(end), r);
    end
end

if isempty(final_msg)
    final_msg = sprintf('Time Limit Reached (t=%.1fs, r=%.1fm)', time(end), r);
end

fprintf('>>> 연산 완료! %s <<<\n', final_msg);

%% 4. 결과 데이터 패키징 (sim_out)
sim_out.time = time;
sim_out.hist_state = hist_state;
sim_out.final_msg = final_msg;

% 초기 상태 패키징
sim_out.init.Xp_i = Xp_i;
sim_out.init.Yp_i = Yp_i;
sim_out.init.Xt_i = Xt_i;
sim_out.init.Yt_i = Yt_i;
sim_out.init.psi_p = psi_p;
sim_out.init.psi_t = psi_t;
sim_out.init.lambda_init = lambda_init;
sim_out.init.r_i = r_i;
sim_out.init.bearing_deg = bearing_deg;

% 상수 패키징
sim_out.param.V_p = V_p;
sim_out.param.V_t = V_t;
sim_out.param.At_constant = At_constant;

%% 5. 플롯 및 데이터 저장 함수 호출
% (이 함수들은 Plotting_function 폴더 안에 생성해야 해)
fprintf('>>> 시각화 및 애니메이션 처리 시작... <<<\n');
Animate_Trajectory(cfg, sim_out);
Plot_Static_Results(cfg, sim_out);

end