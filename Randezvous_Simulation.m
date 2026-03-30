function Randezvous_Simulation(cfg)

%% 1. 파라미터 및 초기 설정
R2D = 180/pi;
D2R = pi/180;

% =========================================================
% [GUIDANCE_MODE 설정]
% =========================================================
GUIDANCE_MODE = cfg.GUIDANCE_MODE; 

% --- Simulation Time ---
dt = cfg.dt_simul;
tf = cfg.tf;
time = 0:dt:tf;

% --- Velocity & Acceleration Setup ---
V_p = cfg.V_p;
V_t = cfg.V_t;
At_constant = cfg.At_constant;
At_cmd = cfg.At_cmd;
At_current = 0;

% --- 화살표 표시 크기 ---
arrow_scale = 7;

% (선택) Success Criteria
r_allow = cfg.r_allow;
th_psi_deg = cfg.th_psi_deg;

%% 2. 초기 상태 정의
% Target의 진행 방향을 항상 +y축에 맞춤

% Target 상태 초기화
Xt_input_km = cfg.Xt_input_km;
Yt_input_km = cfg.Yt_input_km;
psi_ti_deg = cfg.psi_ti_deg;

Xt_i = Xt_input_km * 1000;
Yt_i = Yt_input_km * 1000;
psi_t = psi_ti_deg * D2R;

% Pursuer 상태 초기화
r_from_region_m = cfg.r_from_region_m;
psi_p_from_region = cfg.psi_p_from_region;
bearing_from_region = cfg.bearing_from_region;

r_i = r_from_region_m;
bearing_deg = bearing_from_region;
bearing = bearing_deg * D2R;

% 
rotation_diff_deg = psi_ti_deg - 90.0;
psi_p_deg = psi_p_from_region + rotation_diff_deg;
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
fprintf('Target  Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xt_i, Yt_i, psi_ti_deg);
fprintf('Pursuer Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xp_i, Yp_i, psi_p_deg);

%% 3. [Figure 1, 2] 초기 조건 정적 플롯 (Light Theme 적용)

% --- [사전 계산] ---

lambda_deg_txt = lambda_init * R2D;
psi_p_deg_txt = psi_p * R2D;
psi_t_deg_txt = psi_t * R2D;
sigma_p_deg_txt = (psi_p - lambda_init) * R2D;
sigma_t_deg_txt = (psi_t - lambda_init) * R2D;

% -----------------------------------------------------------
% Figure 1: Global Inertial Frame (Initial)
% -----------------------------------------------------------
figure(1); set(gcf, 'Position', [50 300 500 500], 'Theme', 'light');
hold on; grid on; axis equal;
title('[Fig 1] Initial Condition: Global Inertial Frame', 'Color', 'k');
xlabel('X [m]', 'Color', 'k'); ylabel('Y [m]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

plot([Xp_i Xt_i], [Yp_i Yt_i], 'k--', 'LineWidth', 1); 
quiver(Xp_i, Yp_i, arrow_scale*5*cos(psi_p), arrow_scale*5*sin(psi_p), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
quiver(Xt_i, Yt_i, arrow_scale*5*cos(psi_t), arrow_scale*5*sin(psi_t), 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
plot(Xp_i, Yp_i, 'bo', 'MarkerFaceColor', 'b'); 
plot(Xt_i, Yt_i, 'ro', 'MarkerFaceColor', 'r');

info_p = sprintf('  Pursuer\n  \\psi_p: %.1f\\circ\n  \\sigma_p: %.1f\\circ', psi_p_deg_txt, sigma_p_deg_txt);
text(Xp_i, Yp_i, info_p, 'Color', 'b', 'VerticalAlignment', 'top', 'FontWeight', 'bold'); 

info_t = sprintf('  Target\n  \\psi_t: %.1f\\circ\n  \\sigma_t: %.1f\\circ', psi_t_deg_txt, sigma_t_deg_txt);
text(Xt_i, Yt_i, info_t, 'Color', [0.7 0 0.7], 'VerticalAlignment', 'top', 'FontWeight', 'bold'); 

mid_x = (Xp_i + Xt_i) / 2; mid_y = (Yp_i + Yt_i) / 2;
text(mid_x, mid_y, sprintf('  \\lambda: %.1f\\circ', lambda_deg_txt), ...
    'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

legend('LOS', 'Pursuer Vel', 'Target Vel', 'Location', 'best');

% -----------------------------------------------------------
% Figure 2: Target Body Frame (Initial)
% -----------------------------------------------------------
figure(2); set(gcf, 'Position', [560 300 500 500], 'Theme', 'light');
hold on; grid on; axis equal;
title('[Fig 2] Initial Condition: Target Body Frame (Bearing)', 'Color', 'k');
xlabel('Right [m]', 'Color', 'k'); ylabel('Forward [m]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

rot_angle = (pi/2) - psi_t;
R_mat = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
P_rel_vec = [Xp_i - Xt_i; Yp_i - Yt_i];
P_body = R_mat * P_rel_vec;
psi_p_body = psi_p + rot_angle;

plot([0 0], [0 r_i*1.1], 'k:', 'LineWidth', 0.5); 
plot([0 P_body(1)], [0 P_body(2)], 'Color', [0 0.5 0], 'LineStyle', '--', 'LineWidth', 1); 

quiver(0, 0, 0, arrow_scale*5, 'Color', 'r', 'LineWidth', 3, 'MaxHeadSize',0.5, 'AutoScale','off'); 
quiver(P_body(1), P_body(2), arrow_scale*5*cos(psi_p_body), arrow_scale*5*sin(psi_p_body), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize',0.5, 'AutoScale','off'); 
plot(0, 0, 'ro', 'MarkerFaceColor', 'r'); 
plot(P_body(1), P_body(2), 'bo', 'MarkerFaceColor', 'b');

arc_r = r_i * 0.2;
ang_start = pi/2; ang_end = pi/2 - bearing;
t_arc = linspace(ang_start, ang_end, 30);
plot(arc_r*cos(t_arc), arc_r*sin(t_arc), 'r-', 'LineWidth', 1.5);
text(arc_r*0.7*cos(mean(t_arc)), arc_r*0.7*sin(mean(t_arc)), sprintf('Bearing %.1fdeg', bearing_deg), 'Color', 'r');

if strcmp(GUIDANCE_MODE, 'DPG') && exist('use_auto_angle', 'var') && use_auto_angle
    actual_sigma_init_rad = current_psi_p - lambda_init;
    missile_guidance.sigma_des = actual_sigma_init_rad;
    fprintf('\n>>> [DPG Auto-Mode] 초기 리드각(%.2f deg) 유지 <<<\n', actual_sigma_init_rad * R2D);
end

%% 4. [Figure 3, 4, 5] 애니메이션 초기화 (Light Theme)

arrow_edge_color = 'k'; 

fig3 = figure('Position', [50 50 450 450], 'Name', '3. Target Centered with Saturation Map', 'Theme', 'light');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Target Centered Frame (Acc Saturation)');

% --- 1. Saturation Map (배경) ---
res = 100;
map_range = 200; 
[x_map, y_map] = meshgrid(linspace(-map_range, map_range, res), linspace(-map_range, map_range, res));
R_map = sqrt(x_map.^2 + y_map.^2);
R_map(R_map < 1) = 1; 

Lam_map = atan2(-y_map, -x_map); 
Sig_t_map = (pi/2) - Lam_map; 

h_sat_map = pcolor(x_map, y_map, zeros(size(x_map)));
shading interp; 
set(h_sat_map, 'EdgeColor', 'none', 'FaceAlpha', 0.5); 
colormap(gca, [0.7 0.8 1.0; 1.0 0.7 0.7]); 
clim([0, 1]); 

h_traj_relT = animatedline('Color', 'b', 'LineWidth', 1.5);
h_arrow_T_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], ...
    'FaceColor', 'r', 'EdgeColor', arrow_edge_color); 
h_arrow_P_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], ...
    'FaceColor', 'b', 'EdgeColor', arrow_edge_color); 

% 4. Pursuer Centered View
fig4 = figure('Position', [510 50 450 450], 'Name', '4. Pursuer Centered (Anim)', 'Theme', 'light');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Pursuer Centered Frame');
h_traj_relP = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
h_arrow_P_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', arrow_edge_color);
h_arrow_T_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', arrow_edge_color);

% 5. Inertial Frame View
fig5 = figure('Position', [970 50 450 450], 'Name', '5. Inertial Trajectory (Anim)', 'Theme', 'light');
grid on; axis equal; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Inertial Frame');
h_traj_P_inertial = animatedline('Color', 'b', 'LineWidth', 1.5);
h_traj_T_inertial = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
h_arrow_P_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', arrow_edge_color);
h_arrow_T_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', arrow_edge_color);

figure(fig3); figure(fig4); figure(fig5);
drawnow;

%% 5. 메인 시뮬레이션 루프
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

fprintf('시뮬레이션 준비 중... (그래프 초기화)\n');

relX_T_i = Xp_i - Xt_i; relY_T_i = Yp_i - Yt_i;
relX_P_i = Xt_i - Xp_i; relY_P_i = Yt_i - Yp_i;

V_T_fix = get_arrow_vertices(0, 0, 90*D2R, arrow_scale);
V_P_rel = get_arrow_vertices(relX_T_i, relY_T_i, psi_p, arrow_scale);
set(h_arrow_T_fixed, 'Vertices', V_T_fix); set(h_arrow_P_rel, 'Vertices', V_P_rel);

V_P_fix = get_arrow_vertices(0, 0, 90*D2R, arrow_scale);
V_T_rel = get_arrow_vertices(relX_P_i, relY_P_i, psi_t, arrow_scale);
set(h_arrow_P_fixed, 'Vertices', V_P_fix); set(h_arrow_T_rel, 'Vertices', V_T_rel);

V_P_inert = get_arrow_vertices(Xp_i, Yp_i, psi_p, arrow_scale);
V_T_inert = get_arrow_vertices(Xt_i, Yt_i, psi_t, arrow_scale);
set(h_arrow_P_inertial, 'Vertices', V_P_inert); set(h_arrow_T_inertial, 'Vertices', V_T_inert);

drawnow; 
pause(cfg.pause_t); 

% VDPG 변수
r0 = r_i;                         
sigma_init = psi_p - lambda_init; 

% RDPG 변수
is_stop = 0;

% =========================================================
% 비디오 객체 초기화 (auto_save 켜졌을 때)
% =========================================================

skip_frame = cfg.skip_frame;

if cfg.auto_save == 1
    if ~exist(cfg.save_dir, 'dir')
        mkdir(cfg.save_dir);
        fprintf('\n>>> 저장 폴더 생성됨: %s\n', cfg.save_dir);
    end
    
    % 경로를 합쳐서 비디오 파일 생성
    vidObj3 = VideoWriter(fullfile(cfg.save_dir, 'Figure_3.avi'));
    vidObj4 = VideoWriter(fullfile(cfg.save_dir, 'Figure_4.avi'));
    vidObj5 = VideoWriter(fullfile(cfg.save_dir, 'Figure_5.avi'));
    open(vidObj3);
    open(vidObj4);
    open(vidObj5);
    fprintf('>>> 동영상 녹화 시작. 저장 위치: %s <<<\n', cfg.save_dir);
end

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
    
    % % --- [B] Guidance Command ---
    % ax_t = -At_constant * sin(current_psi_t);
    % ay_t =  At_constant * cos(current_psi_t);
    
    switch GUIDANCE_MODE
        case 'PPNG'
            acc_cmd = missile_guidance.compute_commandPPNG(V_c, lambda_dot);
            At_current = At_constant;
        case 'DPG'
            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
            At_current = At_constant;
        case 'VDPG'
            acc_cmd = missile_guidance.compute_commandVDPG(V_p, ...
                lambda_dot, sigma_p, r, r0, sigma_init);
            At_current = At_constant;
        case 'RDPG'
            [acc_cmd, current_sigma_ref] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
            At_current = At_constant;
        case 'RDPG_T'
            [acc_cmd, current_sigma_ref, tanker_turn_on] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop);
            
            if tanker_turn_on ~= 0
                % 1이면 정방향(At_cmd), -1이면 역방향(-At_cmd)
                At_current = At_cmd * tanker_turn_on; 
            else
                At_current = 0;
            end
        case 'RDPG_SAFE'
            X_state = [r; sigma_t; sigma_p];
            % ~ 대신 current_sigma_ref를 받아줌!
            [acc_cmd, current_sigma_ref, V_clf, H_cbf, slack_opt] = missile_guidance.compute_commandRDPG_SAFE(X_state, lambda_dot);
            At_current = At_constant; % 타겟 등속 기동 유지
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
    
    % --- [D] Animation Update ---

    dx_global = current_Xp - current_Xt;
    dy_global = current_Yp - current_Yt;
    
    if mod(i, skip_frame) == 0 || i == 1


        % Max Acc Region 실시간 업데이트
        Acc_val_map = (V_p^2 ./ R_map) .* (sin(Sig_t_map) - sin(sigma_p));
        Sat_Data = abs(Acc_val_map) > cfg.limit_acc;
        set(h_sat_map, 'CData', double(Sat_Data));

            % --- [Figure 3] 그리기 ---
            rot_ang_T = (pi/2) - current_psi_t; 
            R_mat_T = [cos(rot_ang_T), -sin(rot_ang_T); sin(rot_ang_T), cos(rot_ang_T)];
            pos_P_in_Tframe = R_mat_T * [dx_global; dy_global]; 
            psi_p_in_Tframe = current_psi_p + rot_ang_T;

            addpoints(h_traj_relT, pos_P_in_Tframe(1), pos_P_in_Tframe(2));
            V_P_rel = get_arrow_vertices(pos_P_in_Tframe(1), pos_P_in_Tframe(2), psi_p_in_Tframe, arrow_scale);
            set(h_arrow_P_rel, 'Vertices', V_P_rel);

            % --- [Figure 4] 그리기 ---
            rot_ang_P = (pi/2) - current_psi_p; 
            R_mat_P = [cos(rot_ang_P), -sin(rot_ang_P); sin(rot_ang_P), cos(rot_ang_P)];
            pos_T_in_Pframe = R_mat_P * [-dx_global; -dy_global];
            psi_t_in_Pframe = current_psi_t + rot_ang_P;
            
            addpoints(h_traj_relP, pos_T_in_Pframe(1), pos_T_in_Pframe(2));
            V_T_rel = get_arrow_vertices(pos_T_in_Pframe(1), pos_T_in_Pframe(2), psi_t_in_Pframe, arrow_scale);
            set(h_arrow_T_rel, 'Vertices', V_T_rel);

            % --- [Figure 5] 그리기 ---
            addpoints(h_traj_P_inertial, current_Xp, current_Yp);
            addpoints(h_traj_T_inertial, current_Xt, current_Yt);
            
            V_P_inert = get_arrow_vertices(current_Xp, current_Yp, current_psi_p, arrow_scale);
            V_T_inert = get_arrow_vertices(current_Xt, current_Yt, current_psi_t, arrow_scale);
            set(h_arrow_P_inertial, 'Vertices', V_P_inert);
            set(h_arrow_T_inertial, 'Vertices', V_T_inert);
        
        drawnow limitrate;

        % 비디오 쓰기

        if cfg.auto_save == 1
            currFrame3 = getframe(fig3);
            writeVideo(vidObj3, currFrame3);
            
            currFrame4 = getframe(fig4);
            writeVideo(vidObj4, currFrame4);
            
            currFrame5 = getframe(fig5);
            writeVideo(vidObj5, currFrame5);
        end
    end
    
    % =========================================================
    % [E] Stop Condition
    % =========================================================
    stop_condition = cfg.stop_condition;
    heading_diff_deg = abs(wrapTo180((current_psi_p - current_psi_t) * R2D));

    if ~is_stop
        success_msg = '';
        is_reached = false;

        if V_c < 0 && heading_diff_deg < th_psi_deg && r < r_allow
            success_msg = '랑데부 성공 (CPA 감지, V_c < 0)';
            is_reached = true;
        elseif r < r_allow && heading_diff_deg < th_psi_deg
            success_msg = '랑데부 성공 (r < r_allow)';
            is_reached = true;
        elseif i > 1 && r > hist_state(1, i-1) && r < r_allow && heading_diff_deg < th_psi_deg
            success_msg = sprintf('랑데뷰 성공 (최소거리=%.3fm)', hist_state(1, i));
            is_reached = true;
        end

        if is_reached
            is_stop = 1; 
            print_results_now(time(i), hist_state(:, 1:i), V_p, V_t, R2D, success_msg);
            
            if stop_condition
                hist_state = hist_state(:, 1:i); 
                time = time(1:i);
                break; 
            end
        end
    end

    if i == num_steps && ~is_stop
        msg = sprintf('Time Limit Reached (t=%.1fs, r=%.1fm)', time(end), r);
        print_results_now(time(end), hist_state, V_p, V_t, R2D, msg);
        warning('시뮬레이션 시간 종료! 랑데부/충돌 조건을 만족하지 못했습니다.');
    end
end

if ~stop_condition
    msg = sprintf('Time Limit Reached (t=%.1fs, r=%.1fm)', time(end), r);
    print_results_now(time(end), hist_state, V_p, V_t, R2D, msg);
end

% =========================================================
% [NEW] Figure 3 Update: Draw Final DPG Region
% =========================================================
% if contains(GUIDANCE_MODE, 'RDPG')
%     % 시뮬레이션 종료 후, 마지막 sigma_ref 값을 가져와서 그리기
%     % hist_state의 9번째 행이 sigma_ref (RDPG/RDPG_T 공통)
%     last_sigma_ref = hist_state(9, end);
% 
%     figure(fig3); % Figure 3 활성화
%     hold on;
% 
%     % draw_DPG_Region 호출 (Target Centered Frame은 Target Heading이 90도이므로 그대로 사용 가능)
%     % 절댓값(abs)을 사용하여 대칭적인 영역을 그림
%     draw_DPG_Region(gca, rad2deg(abs(last_sigma_ref)), cfg.r_allow);
% 
%     fprintf('>>> [Figure 3] Final DPG Region Drawn (Ref Sigma: %.2f deg) <<<\n', last_sigma_ref * R2D);
%     xlim([-800 10])
%     ylim([-400 400])
% 
%     % 비디오 객체 닫기
%     if cfg.auto_save == 1
%         close(vidObj3);
%         close(vidObj4);
%         close(vidObj5);
%         fprintf('>>> 동영상 저장 완료. <<<\n');
%     end
% 
% end


%% 6. [Figure 6, 7, 8] 결과 분석 플롯 통합 (Light Theme)
% 흩어져있던 Figure들을 세로 배열로 깔끔하게 정리

r_data = hist_state(1,:);
lambda_data = hist_state(2,:) * R2D;
psi_p_data = hist_state(3,:) * R2D; 
psi_t_data = hist_state(4,:) * R2D;
sigma_p_data = hist_state(5,:) * R2D;
sigma_t_data = hist_state(6,:) * R2D;
Vc_data = hist_state(7,:);
acc_p_data = hist_state(8,:);

% =========================================================
% [Figure 6] Distance, V_c, Acc vs Time, Acc vs Range
% =========================================================
fig6 = figure(6); set(fig6, 'Position', [10, 50, 500, 750], 'Theme', 'light');

% 6-1: Distance
subplot(4,1,1);
plot(time, r_data, 'r-', 'LineWidth', 2); hold on; grid on;
[~, idx_208] = min(abs(r_data - 2.08)); 
t_208 = time(idx_208); r_actual_208 = r_data(idx_208);
plot(t_208, r_actual_208, 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
text(t_208, r_actual_208, sprintf('  t = %.2f s (%.2f m)', t_208, r_actual_208), ...
    'FontSize', 10, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'Color', 'b');
title('[Fig 6-1] Relative Distance', 'Color', 'k');
ylabel('Range [m]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 6-2: dot_r (부호 뒤집음)
subplot(4,1,2);
plot(time, -Vc_data, 'b-', 'LineWidth', 2); grid on;
title('[Fig 6-2] Closing Velocity ($\dot{r}$)', 'Interpreter','latex','Color', 'k');
ylabel('Speed [m/s]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 6-3: Acc vs time
subplot(4,1,3);
plot(time, acc_p_data, 'r-', 'LineWidth', 2); grid on;
title('[Fig 6-3] Pursuer Acceleration Command', 'Color', 'k');
ylabel('Acc [m/s^2]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
max_acc = max(abs(acc_p_data));
text(time(end)*0.5, max_acc*0.9, sprintf('Max: %.2f m/s^2', max_acc), ...
    'Color', [0 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');

% 6-4: Acc vs range 
subplot(4,1,4);
plot(r_data, acc_p_data, 'b-', 'LineWidth', 2); grid on;
set(gca, 'XDir', 'reverse'); 
title('[Fig 6-4] Acc vs Range (Range-to-Go)', 'Color', 'k');
xlabel('Relative Distance r [m]', 'Color', 'k'); ylabel('Acc [m/s^2]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
text(r_data(1)*0.5, max_acc*0.9, sprintf('Max: %.2f m/s^2', max_acc), ...
    'Color', [0 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');

% =========================================================
% [Figure 7] LOS, Lead Angle, Heading angle, Control Energy
% =========================================================
fig7 = figure(7); set(fig7, 'Position', [520, 50, 500, 900], 'Theme', 'light');

% 7-1: LOS
subplot(4,1,1);
plot(time, lambda_data, 'm-', 'LineWidth', 2); grid on;
title('[Fig 7-1] LOS Angle (\lambda)', 'Color', 'k');
ylabel('Angle [deg]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 7-2: Lead Angle
subplot(4,1,2); hold on; grid on;
if contains(GUIDANCE_MODE, 'RDPG')
    sigma_ref_data = hist_state(9,:) * R2D; 
    plot(time, sigma_p_data, 'b-', 'LineWidth', 1.5);
    plot(time, sigma_t_data, 'r--', 'LineWidth', 1.5);
    plot(time, sigma_ref_data, 'g:', 'LineWidth', 2.0);
    legend({'\sigma_p', '\sigma_t', '\sigma_{ref}'}, 'Location', 'best', 'TextColor', 'k');
    title('[Fig 7-2] RDPG Lead Angle Tracking', 'Color', 'k');
else
    plot(time, sigma_p_data, 'b-', 'LineWidth', 1.5);
    plot(time, sigma_t_data, 'r--', 'LineWidth', 1.5);
    legend({'\sigma_p', '\sigma_t'}, 'TextColor', 'k', 'Location', 'best'); 
    title('[Fig 7-2] Lead Angles (\sigma)', 'Color', 'k');
end
ylabel('Angle [deg]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 7-3: Heading Angle
subplot(4,1,3);
plot(time, psi_p_data, 'b-', 'LineWidth', 2); hold on;
plot(time, psi_t_data, 'r-', 'LineWidth', 1.5); grid on;
title('[Fig 7-3] Heading Angles (\psi)', 'Color', 'k');
ylabel('Angle [deg]', 'Color', 'k');
legend({'Pursuer (\psi_p)', 'Target (\psi_t)'}, 'TextColor', 'k', 'Location', 'best');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% -----------------------------------------------------
% [수정됨] 7-4: Control Energy (가속도 a^2 적분) - 모든 모드 공통
% -----------------------------------------------------
subplot(4,1,4); hold on; grid on;

% Pursuer 에너지 (청록색) - 무차원 u 대신 실제 가속도 a_p 사용
energy_p = cumtrapz(time, acc_p_data.^2);
plot(time, energy_p, 'c-', 'LineWidth', 2);

% Target 가속도 데이터 추출 (RDPG_T는 가변, 나머지는 상수)
if strcmp(GUIDANCE_MODE, 'RDPG_T')
    acc_t_data = hist_state(11, :);
else
    acc_t_data = At_constant * ones(size(time));
end

% Target 에너지 (보라색) - 실제 가속도 a_t 사용
energy_t = cumtrapz(time, acc_t_data.^2);

% 타겟이 한 번이라도 기동했으면 그래프에 표시
if any(acc_t_data ~= 0)
    plot(time, energy_t, 'Color', [0.5, 0, 0.5], 'LineWidth', 2); 
    legend({'Pursuer ($a_p^2$)', 'Target ($a_t^2$)'}, 'Interpreter', 'latex', 'Location', 'best', 'TextColor', 'k');
    total_energy_str = sprintf('Total Energy\n  P: %.2f\n  T: %.2f', energy_p(end), energy_t(end));
else
    legend({'Pursuer ($a_p^2$)'}, 'Interpreter', 'latex', 'Location', 'best', 'TextColor', 'k');
    total_energy_str = sprintf('Total P: %.2f', energy_p(end));
end

title('[Fig 7-4] Control Energy ($\int a^2 dt$)', 'Interpreter', 'latex', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); 
ylabel('Energy $[m^2/s^3]$', 'Interpreter', 'latex', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 텍스트로 최종 에너지 값 눈에 띄게 표시
x_pos = time(end) * 0.02;
y_max = max([max(energy_p), max(energy_t)]); % P와 T 중 더 큰 에너지 기준
y_pos = y_max * 0.8;
if y_pos == 0, y_pos = 0.5; end % 에너지가 0일 경우(초기화)를 대비한 예외 처리
text(x_pos, y_pos, total_energy_str, 'Color', 'k', 'FontWeight', 'bold', ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 2);


% =========================================================
% [NEW! Figure 8] RDPG_SAFE 전용: V(x), H(x), Slack 플롯
% =========================================================
if strcmp(GUIDANCE_MODE, 'RDPG_SAFE')
    % 창 세로 길이를 3개의 그래프에 맞춰 750으로 세팅
    fig8 = figure(8); set(fig8, 'Position', [1030, 50, 500, 750], 'Theme', 'light');
    
    V_data = hist_state(10, :);
    H_data = hist_state(11, :);
    slack_data = hist_state(12, :);
    
    % -----------------------------------------------------
    % 8-1: CLF V(x)
    % -----------------------------------------------------
    subplot(3, 1, 1);
    plot(time, V_data, 'b-', 'LineWidth', 2); grid on;
    title('[Fig 8-1] CLF: $V(x) = (\sigma_p - \sigma_{ref})^2$', 'Interpreter', 'latex', 'Color', 'k');
    ylabel('$V(x)$', 'Interpreter', 'latex', 'Color', 'k');
    set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
    
    % -----------------------------------------------------
    % 8-2: CBF H(x) (Margin)
    % -----------------------------------------------------
    subplot(3, 1, 2);
    plot(time, H_data, 'r-', 'LineWidth', 2); hold on; grid on;
    yline(0, 'k--', 'LineWidth', 1.5); 
    title('[Fig 8-2] CBF Margin: $h(x) = a_{max} - |a_{geom}|$', 'Interpreter', 'latex', 'Color', 'k');
    ylabel('$h(x)$', 'Interpreter', 'latex', 'Color', 'k');
    set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
    
    % -----------------------------------------------------
    % 8-3: Slack Variable
    % -----------------------------------------------------
    subplot(3, 1, 3);
    plot(time, slack_data, 'm-', 'LineWidth', 2); grid on;
    title('[Fig 8-3] Slack Variable: $\delta$', 'Interpreter', 'latex', 'Color', 'k');
    xlabel('Time [s]', 'Color', 'k'); ylabel('$\delta$', 'Interpreter', 'latex', 'Color', 'k');
    set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
end

%% [Helper Functions]
function V_rot = get_arrow_vertices(x, y, psi, scale)
    V_base = [ 1.5,  0.0; -1.0,  0.8; -0.5,  0.0; -1.0, -0.8] * scale;
    R_mat = [cos(psi), -sin(psi); sin(psi),  cos(psi)];
    V_rot = (R_mat * V_base')'; 
    V_rot(:,1) = V_rot(:,1) + x; 
    V_rot(:,2) = V_rot(:,2) + y; 
end

function print_results_now(tf, hist, Vp, ~, R2D, msg)
    rf = hist(1, end);      
    Vc = hist(7, end);      
    sigmap = hist(5, end);  
    sigmat = hist(6, end);  
    sigma_diff = sigmap - sigmat;
    distp = Vp * tf;

    fprintf('\n======================================================\n');
    fprintf('시뮬레이션 종료: %s\n', msg);
    fprintf('======================================================\n');
    fprintf('1. 최종 시간 (t_f)      : %.4f s\n', tf);
    fprintf('2. 최종 거리 r(t_f)     : %.4f m\n', rf);
    fprintf('3. 최종 속도 V_c        : %.4f m/s\n', Vc);
    fprintf('4. 리드각 오차 (Sig_p - Sig_t): %.4f deg\n', sigma_diff * R2D);
    fprintf('5. 총 이동 거리 (Pursuer): %.2f m\n', distp);
    fprintf('======================================================\n');
end

%% 7. [Auto Save] 결과 이미지 자동 저장

auto_save = cfg.auto_save;

if auto_save

    % 하드코딩된 경로 삭제하고 cfg에서 받아온 경로 사용
    save_folder = cfg.save_dir; 
    
    % 비디오 녹화에서 이미 만들었겠지만, 안전제일! 한 번 더 체크~
    if ~exist(save_folder, 'dir')
        mkdir(save_folder);
        fprintf('\n>>> 저장 폴더 생성됨: %s\n', save_folder);
    end

    % 창이 통합되어서 이제 1~8번 Figure만 저장하면 됨!
    target_figs = 1:8;

    fprintf('>>> 그래프 이미지 저장 시작...\n');

    for idx = target_figs
        if ishandle(idx)
            h_fig = figure(idx);
            filename = sprintf('Fig%02d_Result.png', idx);
            full_path = fullfile(save_folder, filename);
            print(h_fig, full_path, '-dpng', '-r150'); 
            fprintf('    [저장 완료] %s\n', filename);
        end
    end

    fprintf('>>> 저장 끝\n');
end
end