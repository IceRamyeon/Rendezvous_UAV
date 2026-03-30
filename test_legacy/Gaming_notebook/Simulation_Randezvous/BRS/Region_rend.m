function Region_rend()
%% Region_rend.m
% 이론적 DPG 영역 내부에 있는 점들에 대해 궤적을 추출하고,
% Max Acc Region (이론적 최대 가속도 요구량) 침범 시 실패(Fail) 처리하도록 수정했어~

close all; 
clc; 

%% 1. 통합 파라미터 초기화 블록
GUIDANCE_Switch = 2;    % 1=PPNG, 2=DPG, 3=VDPG, 4=RDPG, 5=RDPG_T, 6=RDPG_SAFE
switch GUIDANCE_Switch
    case 1, GUIDANCE_MODE = 'PPNG';
    case 2, GUIDANCE_MODE = 'DPG';
    case 3, GUIDANCE_MODE = 'VDPG';
    case 4, GUIDANCE_MODE = 'RDPG';
    case 5, GUIDANCE_MODE = 'RDPG_T';
    case 6, GUIDANCE_MODE = 'RDPG_SAFE';
end

V_p = 20.0;                     
V_t = 20.0;                     
target_turn_rate_deg = 0.0;     
At_constant = V_t * deg2rad(target_turn_rate_deg);
limit_acc = 1;                  % [G]
g = 9.81;                       % ★ [추가] 중력가속도
dt = 0.01;                      
tf = 100;                       

N_gain = 3.0;                   
bias_acc = -0.0;                

gain_k = 5.0;                   
target_lead_angle_deg = 60.0;   

r_allow = 2;                    
r_theory_limit = r_allow;       
th_psi_deg = 5.0;               
rate_limit_degree = 15;         
rate_limit = deg2rad(rate_limit_degree);
alpha = 1;                      

target_turn_rate_cmd_deg = 0.4;
At_cmd = V_t * deg2rad(target_turn_rate_cmd_deg);
RDPG_T_MODE = 1;                
sigma_offset = 9;               
RDPG_test = 1;                  

r_min_km = 0.1; r_max_km = 1; r_step_km = 0.002;
b_min_deg = -90; b_max_deg = -0; b_step_deg = 0.05;
sigma_cases_deg = [0 15 30 45 60 75];

fprintf('>>> [Reachability Region : "%s"] 궤적 추출 및 Max Acc 검사 시작할게! 으헤~\n', GUIDANCE_MODE);

%% 2. 시뮬레이션 준비
R2D = 180/pi;
D2R = pi/180;

switch GUIDANCE_MODE
    case 'PPNG'
        missile_guidance = PPNG(N_gain, bias_acc, limit_acc); 
    case 'DPG'
        missile_guidance = DPG(target_lead_angle_deg, gain_k, limit_acc);
    case 'VDPG'
        missile_guidance = VDPG(gain_k, limit_acc);
    case 'RDPG'
        missile_guidance = RDPG(gain_k, limit_acc, r_allow, rate_limit, dt, 0, alpha, RDPG_test);
    case 'RDPG_T'
        missile_guidance = RDPG_T(gain_k, limit_acc, r_allow, rate_limit, dt, 0, RDPG_T_MODE, alpha, sigma_offset, RDPG_test);
    otherwise
        error('으헤... 모드 설정이 잘못된 것 같아.');
end

r_vec = (r_min_km : r_step_km : r_max_km) * 1000; 
b_vec = (b_min_deg : b_step_deg : b_max_deg) * D2R; 

All_Candidates_Data = {}; 
collected_data = [];

%% 3. 플롯 준비
figure('Position', [100 50 1400 900], 'Theme', 'light'); 
t_plot = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

switch GUIDANCE_MODE
    case 'PPNG'
        title_str = sprintf('[Reachability Map] Mode: %s (N=%.1f)', GUIDANCE_MODE, N_gain);
    case 'VDPG'
        title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f)', GUIDANCE_MODE, gain_k);
    case 'DPG'
        title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f)', GUIDANCE_MODE, gain_k);
    case {'RDPG', 'RDPG_T'}
        title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f, \\alpha=%.1f)', GUIDANCE_MODE, gain_k, alpha);
end
title(t_plot, title_str, 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');

Xt_i = 0; Yt_i = 0;
psi_t = 90 * D2R;

%% 4. 케이스별 루프 시작
for s_idx = 1:length(sigma_cases_deg)
    current_sigma_deg = sigma_cases_deg(s_idx);
    current_sigma_rad = current_sigma_deg * D2R;
    
    if strcmp(GUIDANCE_MODE, 'DPG')
        missile_guidance.sigma_des = current_sigma_rad; 
    end
    
    fprintf('>>> 분석 진행 중: 초기 리드각 %d도 (%d/%d) <<<\n', current_sigma_deg, s_idx, length(sigma_cases_deg));
    
    Suc_X = []; Suc_Y = [];
    Fail_X = []; Fail_Y = [];
    
    for r_idx = 1:length(r_vec)
        r_start = r_vec(r_idx);
        for b_idx = 1:length(b_vec)
            bearing = b_vec(b_idx);
            
            theta_global = psi_t - bearing; 
            Xp_i = Xt_i + r_start * cos(theta_global);
            Yp_i = Yt_i + r_start * sin(theta_global);
            
            dx = Xt_i - Xp_i; dy = Yt_i - Yp_i;
            lambda_init = atan2(dy, dx);
            init_psi_p = lambda_init + current_sigma_rad; 
            
            % --- 이론적 DPG 영역 내부 점만 고르기 ---
            sigma_t0_check = psi_t - lambda_init;
            sigma_t0_check = atan2(sin(sigma_t0_check), cos(sigma_t0_check)); 
            
            is_inside_theory = false;
            sigma_pf = current_sigma_rad;
            
            if (sigma_t0_check >= sigma_pf) && (sigma_t0_check <= pi - sigma_pf)
                numerator = r_theory_limit * cos(sigma_pf)^2;
                denominator = cos((sigma_t0_check + sigma_pf)/2)^2;
                r_boundary = numerator / denominator;
                
                if r_start <= r_boundary
                    is_inside_theory = true;
                end
            end
            
            if ~is_inside_theory
                continue; 
            end
            % ----------------------------------------

            cur_Xp = Xp_i; cur_Yp = Yp_i;
            cur_Xt = Xt_i; cur_Yt = Yt_i;
            cur_psi_p = init_psi_p;
            cur_psi_t = psi_t;
            
            if strcmp(GUIDANCE_MODE, 'RDPG') || strcmp(GUIDANCE_MODE, 'RDPG_T')
                 missile_guidance.sigma_ref_prev = current_sigma_rad;
            end
            
            is_stop = 0; 
            is_success = false; 
            is_acc_safe = true;     % ★ [추가] 가속도 포화 안전 여부
            sim_time = 0;
            
            temp_traj = zeros(ceil(tf/dt) + 1, 3);
            step_idx = 0;
            
            % --- Trajectory Integration ---
            while sim_time < tf
                Rx = cur_Xt - cur_Xp; Ry = cur_Yt - cur_Yp;
                r = sqrt(Rx^2 + Ry^2);
                lambda = atan2(Ry, Rx);
                
                sigma_p = cur_psi_p - lambda;
                sigma_t = cur_psi_t - lambda;
                
                % ★ [추가] Max Acc Region 침범 검사
                % a = (V^2 / r) * (sin(sigma_t) - sin(sigma_p))
                a_req = (V_p^2 / r) * (sin(sigma_t) - sin(sigma_p));
                
                if abs(a_req) > limit_acc * g
                    is_acc_safe = false; % 제한 넘으면 즉시 실패!
                    break; 
                end
                
                % 궤적 데이터 기록 (r, pi-sigma_t, sigma_p)
                step_idx = step_idx + 1;
                temp_traj(step_idx, 1) = r;
                temp_traj(step_idx, 2) = pi - sigma_t; 
                temp_traj(step_idx, 3) = sigma_p;
                
                r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
                V_c = -r_dot;
                lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
                
                heading_diff_deg = abs(mod((cur_psi_p - cur_psi_t)*R2D + 180, 360) - 180);
                if (r <= r_allow) && (heading_diff_deg <= th_psi_deg)
                    is_success = true;
                    break; 
                end
                
                if r > 1100 
                    break; 
                end
                
                At_current = At_constant; 
                switch GUIDANCE_MODE
                    case 'PPNG'
                        acc_cmd = missile_guidance.compute_commandPPNG(V_c, lambda_dot);
                    case 'DPG'
                        acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
                    case 'VDPG'
                        acc_cmd = missile_guidance.compute_commandVDPG(V_p, lambda_dot, sigma_p, r, r_start, current_sigma_rad);
                    case 'RDPG'
                        [acc_cmd, ~] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
                    case 'RDPG_T'
                        [acc_cmd, ~, tanker_turn] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop);
                        if tanker_turn ~= 0
                            At_current = At_cmd * tanker_turn;
                        else
                            At_current = 0;
                        end
                end
                
                cur_psi_p = cur_psi_p + (acc_cmd/V_p)*dt;
                cur_psi_t = cur_psi_t + (At_current/V_t)*dt;
                
                cur_Xp = cur_Xp + V_p * cos(cur_psi_p) * dt;
                cur_Yp = cur_Yp + V_p * sin(cur_psi_p) * dt;
                cur_Xt = cur_Xt + V_t * cos(cur_psi_t) * dt;
                cur_Yt = cur_Yt + V_t * sin(cur_psi_t) * dt;
                
                sim_time = sim_time + dt;
            end
            
            % ★ 성공 조건은 기존 조건 달성(is_success) AND 가속도 포화 안 됨(is_acc_safe)
            final_success = is_success && is_acc_safe;
            
            cand_data.Initial_Range = r_start;
            cand_data.Initial_Bearing = bearing * R2D;
            cand_data.Sigma0_Case = current_sigma_deg;
            cand_data.Is_Success = final_success;
            cand_data.Trajectory = temp_traj(1:step_idx, :); 
            
            All_Candidates_Data{end+1} = cand_data; 
            
            % 결과 플롯 저장 (최종 성공/실패)
            if final_success
                Suc_X(end+1) = Xp_i; Suc_Y(end+1) = Yp_i;
                collected_data = [collected_data; [r_start, init_psi_p * R2D, bearing * R2D, current_sigma_deg]];
            else
                Fail_X(end+1) = Xp_i; Fail_Y(end+1) = Yp_i;
            end
        end
    end
    
   % --- 서브플롯 그리기 ---
    ax = nexttile;
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.2);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

    if ~isempty(Fail_X), plot(ax, Fail_X, Fail_Y, '.', 'MarkerSize', 5, 'Color', [1 0.4 0.4]); end
    if ~isempty(Suc_X), plot(ax, Suc_X, Suc_Y, '.', 'MarkerSize', 6, 'Color', 'b'); end
    plot(ax, 0, 0, '^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'Color', 'g');

    switch GUIDANCE_MODE
        case 'PPNG'
            if exist('draw_PPNG_Region', 'file')
                draw_PPNG_Region(ax, current_sigma_rad, r_theory_limit, N_gain);
            end
        case 'DPG'
            if exist('draw_DPG_Region', 'file')
                draw_DPG_Region(ax, current_sigma_rad, r_theory_limit);
            end
        case {'RDPG', 'RDPG_T'}
             if exist('draw_DPG_Region', 'file')
                draw_DPG_Region(ax, current_sigma_rad, r_theory_limit);
             end
    end
    
    % ★ [아저씨가 추가한 범례(Legend) 코드] ★
    % 빈 데이터(NaN)를 이용해 범례용 더미 핸들을 만들어서 깔끔하게 띄워줄게~
    h_safe = plot(ax, NaN, NaN, '.', 'MarkerSize', 15, 'Color', 'b');
    h_unsafe = plot(ax, NaN, NaN, '.', 'MarkerSize', 15, 'Color', [1 0.4 0.4]);
    h_reach = patch(ax, NaN, NaN, 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % 노란색 영역 표시용
    
    % 범례 추가 (위치는 오른쪽 위가 무난할 거야)
    legend(ax, [h_safe, h_unsafe, h_reach], {'safe', 'unsafe', 'Reachable'}, ...
           'Location', 'northeast', 'FontSize', 10);
    % ★★★★★★★★★★★★★★★★★★★★★★★★★★★
    
    title(ax, sprintf('\\sigma_{p0} = %d^\\circ', current_sigma_deg), 'Color', 'k');
    if s_idx > 3, xlabel('East [m]', 'Color', 'k'); end
    if mod(s_idx, 3) == 1, ylabel('North [m]', 'Color', 'k'); end
    
    draw_max = abs(r_max_km) * 1000 + 100;
    xlim([-draw_max 10]); ylim([-10 draw_max]); 
    drawnow;
end