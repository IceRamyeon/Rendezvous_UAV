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
At_cmd = cfg.At_cmd;

% --- 화살표 표시 크기 ---
arrow_scale = 15;

% (선택) Success Criteria
r_allow = cfg.r_allow;
th_psi_deg = cfg.th_psi_deg;

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
            missile_guidance = RDPG(cfg.gain_k, cfg.limit_acc, cfg.r_allow, cfg.rate_limit, dt, 0);
        otherwise
            error('모드 없음');
    end
    

%% 2. 초기 상태 정의

Xt_input_km = cfg.Xt_input_km;
Yt_input_km = cfg.Yt_input_km;
psi_ti_deg = cfg.psi_ti_deg;

r_from_region_m = cfg.r_from_region_m;
psi_p_from_region = cfg.psi_p_from_region;
bearing_from_region = cfg.bearing_from_region;

Xt_i = Xt_input_km * 1000;
Yt_i = Yt_input_km * 1000;
psi_t = psi_ti_deg * D2R;

r_i = r_from_region_m;
bearing_deg = bearing_from_region;
bearing = bearing_deg * D2R;

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

fprintf('>>> 변환 및 초기화 완료 <<<\n');
fprintf('Target  Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xt_i, Yt_i, psi_ti_deg);
fprintf('Pursuer Pos: [%.1f, %.1f] m, Heading: %.1f deg\n', Xp_i, Yp_i, psi_p_deg);

%% 3. [Figure 1, 2] 초기 조건 정적 플롯 (Light Theme 적용)

% --- [사전 계산] ---
dx = Xt_i - Xp_i;
dy = Yt_i - Yp_i;
lambda_init = atan2(dy, dx);

lambda_deg_txt = lambda_init * R2D;
psi_p_deg_txt = psi_p * R2D;
psi_t_deg_txt = psi_t * R2D;
sigma_p_deg_txt = (psi_p - lambda_init) * R2D;
sigma_t_deg_txt = (psi_t - lambda_init) * R2D;

% -----------------------------------------------------------
% Figure 1: Global Inertial Frame (Initial)
% -----------------------------------------------------------
% [수정] 배경 흰색('w'), 텍스트 검은색('k')으로 변경
figure(1); set(gcf, 'Position', [50 300 500 500], 'Color', 'w');
hold on; grid on; axis equal;
title('[Fig 1] Initial Condition: Global Inertial Frame', 'Color', 'k');
xlabel('X [m]', 'Color', 'k'); ylabel('Y [m]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% 1. 기본 요소 그리기
% [수정] LOS 라인을 노란색('y')에서 검은색 점선('k--')으로 변경
plot([Xp_i Xt_i], [Yp_i Yt_i], 'k--', 'LineWidth', 1); 
quiver(Xp_i, Yp_i, arrow_scale*5*cos(psi_p), arrow_scale*5*sin(psi_p), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
quiver(Xt_i, Yt_i, arrow_scale*5*cos(psi_t), arrow_scale*5*sin(psi_t), 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
plot(Xp_i, Yp_i, 'bo', 'MarkerFaceColor', 'b'); 
plot(Xt_i, Yt_i, 'ro', 'MarkerFaceColor', 'r');

% 2. 텍스트 정보 표시
% [수정] 텍스트 색상 변경 (Cyan->Blue, Magenta->DarkMagenta, Yellow->Black)
info_p = sprintf('  Pursuer\n  \\psi_p: %.1f\\circ\n  \\sigma_p: %.1f\\circ', psi_p_deg_txt, sigma_p_deg_txt);
text(Xp_i, Yp_i, info_p, 'Color', 'b', 'VerticalAlignment', 'top', 'FontWeight', 'bold'); 

info_t = sprintf('  Target\n  \\psi_t: %.1f\\circ\n  \\sigma_t: %.1f\\circ', psi_t_deg_txt, sigma_t_deg_txt);
text(Xt_i, Yt_i, info_t, 'Color', [0.7 0 0.7], 'VerticalAlignment', 'top', 'FontWeight', 'bold'); % 보라색 계열

mid_x = (Xp_i + Xt_i) / 2; mid_y = (Yp_i + Yt_i) / 2;
text(mid_x, mid_y, sprintf('  \\lambda: %.1f\\circ', lambda_deg_txt), ...
    'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

% [수정] 범례 텍스트 색상 옵션 제거 (자동으로 검은색 됨)
legend('LOS', 'Pursuer Vel', 'Target Vel', 'Location', 'best');

% -----------------------------------------------------------
% Figure 2: Target Body Frame (Initial)
% -----------------------------------------------------------
% [수정] 배경 흰색
figure(2); set(gcf, 'Position', [560 300 500 500], 'Color', 'w');
hold on; grid on; axis equal;
title('[Fig 2] Initial Condition: Target Body Frame (Bearing)', 'Color', 'k');
xlabel('Right [m]', 'Color', 'k'); ylabel('Forward [m]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

% Body Frame 변환
rot_angle = (pi/2) - psi_t;
R_mat = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
P_rel_vec = [Xp_i - Xt_i; Yp_i - Yt_i];
P_body = R_mat * P_rel_vec;
psi_p_body = psi_p + rot_angle;

% Plot
% [수정] 안 보이는 노란색 점선 -> 검은색 점선('k:')
plot([0 0], [0 r_i*1.1], 'k:', 'LineWidth', 0.5); 
% [수정] 녹색은 흰 배경에 잘 보이므로 유지하되 조금 진하게 (옵션)
plot([0 P_body(1)], [0 P_body(2)], 'Color', [0 0.5 0], 'LineStyle', '--', 'LineWidth', 1); 

quiver(0, 0, 0, arrow_scale*5, 'Color', 'r', 'LineWidth', 3, 'MaxHeadSize',0.5, 'AutoScale','off'); 
quiver(P_body(1), P_body(2), arrow_scale*5*cos(psi_p_body), arrow_scale*5*sin(psi_p_body), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize',0.5, 'AutoScale','off'); 
plot(0, 0, 'ro', 'MarkerFaceColor', 'r'); 
plot(P_body(1), P_body(2), 'bo', 'MarkerFaceColor', 'b');

% Bearing Arc
arc_r = r_i * 0.2;
ang_start = pi/2; ang_end = pi/2 - bearing;
t_arc = linspace(ang_start, ang_end, 30);
plot(arc_r*cos(t_arc), arc_r*sin(t_arc), 'r-', 'LineWidth', 1.5);
text(arc_r*0.7*cos(mean(t_arc)), arc_r*0.7*sin(mean(t_arc)), sprintf('Bearing %.1fdeg', bearing_deg), 'Color', 'r');

% =========================================================
% [DPG Auto-Set Logic] 
% =========================================================
if strcmp(GUIDANCE_MODE, 'DPG') && exist('use_auto_angle', 'var') && use_auto_angle
    actual_sigma_init_rad = current_psi_p - lambda_init;
    missile_guidance.sigma_des = actual_sigma_init_rad;
    fprintf('\n>>> [DPG Auto-Mode] 초기 리드각(%.2f deg) 유지 <<<\n', actual_sigma_init_rad * R2D);
end

%% 4. [Figure 3, 4, 5] 애니메이션 초기화 (Light Theme)

% [수정] 화살표 테두리 색: 초록(g) -> 검정(k) (흰 배경 가시성 확보)
arrow_edge_color = 'k'; 

% 3. Target Centered View
fig3 = figure('Position', [50 50 400 400], 'Name', '3. Target Centered (Anim)', 'Color', 'w');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Target Centered Frame');
h_traj_relT = animatedline('Color', 'b', 'LineWidth', 1.5);
h_arrow_T_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', arrow_edge_color); 
h_arrow_P_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', arrow_edge_color);

% 4. Pursuer Centered View
fig4 = figure('Position', [460 50 400 400], 'Name', '4. Pursuer Centered (Anim)', 'Color', 'w');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Pursuer Centered Frame');
h_traj_relP = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
h_arrow_P_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', arrow_edge_color);
h_arrow_T_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', arrow_edge_color);

% 5. Inertial Frame View
fig5 = figure('Position', [870 200 400 400], 'Name', '5. Inertial Trajectory (Anim)', 'Color', 'w');
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
hist_state = zeros(8, num_steps); 

fprintf('시뮬레이션 준비 중... (그래프 초기화)\n');

relX_T_i = Xp_i - Xt_i; relY_T_i = Yp_i - Yt_i;
relX_P_i = Xt_i - Xp_i; relY_P_i = Yt_i - Yp_i;

% 초기 화살표 업데이트
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

% RDPG 초기값
if strcmp(GUIDANCE_MODE, 'RDPG')
    current_sigma_p = current_psi_p - lambda_init;
    missile_guidance.sigma_ref_prev = current_sigma_p; 
end

for i = 1:num_steps
    
    % --- [A] Navigation ---
    Rx = current_Xt - current_Xp;
    Ry = current_Yt - current_Yp;
    r = sqrt(Rx^2 + Ry^2);
    lambda = atan2(Ry, Rx);
    
    sigma_p = current_psi_p - lambda;
    sigma_t = current_psi_t - lambda;
    
    % Closing Velocity & LOS Rate
    r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
    V_c = -r_dot;
    lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
    
    % --- [B] Guidance Command ---
    ax_t = -At_cmd * sin(current_psi_t);
    ay_t =  At_cmd * cos(current_psi_t);
    
    switch GUIDANCE_MODE
        case 'PPNG'
            acc_cmd = missile_guidance.compute_commandPPNG(V_c, lambda_dot);
        case 'DPG'
            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
        case 'VDPG'
            acc_cmd = missile_guidance.compute_commandVDPG(V_p, ...
                lambda_dot, sigma_p, r, r0, sigma_init);
        case 'RDPG'
            [acc_cmd, ~] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
    end
    
    % --- [C] Kinematics Update ---
    omega_p = acc_cmd / V_p;
    current_psi_p = current_psi_p + omega_p * dt;
    current_Xp = current_Xp + V_p * cos(current_psi_p) * dt;
    current_Yp = current_Yp + V_p * sin(current_psi_p) * dt;
    
    omega_t = At_cmd / V_t;
    current_psi_t = current_psi_t + omega_t * dt;
    current_Xt = current_Xt + V_t * cos(current_psi_t) * dt;
    current_Yt = current_Yt + V_t * sin(current_psi_t) * dt;
    
    % Data Logging
    hist_state(:, i) = [r; lambda; current_psi_p; current_psi_t; sigma_p; sigma_t; V_c; acc_cmd];
    
    % --- [D] Animation Update ---
    if mod(i, 1) == 0
        dx_global = current_Xp - current_Xt;
        dy_global = current_Yp - current_Yt;

        % [Figure 3] Target Centered
        rot_ang_T = (pi/2) - current_psi_t; 
        R_mat_T = [cos(rot_ang_T), -sin(rot_ang_T); sin(rot_ang_T), cos(rot_ang_T)];
        
        pos_P_in_Tframe = R_mat_T * [dx_global; dy_global]; 
        psi_p_in_Tframe = current_psi_p + rot_ang_T;

        addpoints(h_traj_relT, pos_P_in_Tframe(1), pos_P_in_Tframe(2));
        V_P_rel = get_arrow_vertices(pos_P_in_Tframe(1), pos_P_in_Tframe(2), psi_p_in_Tframe, arrow_scale);
        set(h_arrow_P_rel, 'Vertices', V_P_rel);

        % [Figure 4] Pursuer Centered
        rot_ang_P = (pi/2) - current_psi_p; 
        R_mat_P = [cos(rot_ang_P), -sin(rot_ang_P); sin(rot_ang_P), cos(rot_ang_P)];
        
        pos_T_in_Pframe = R_mat_P * [-dx_global; -dy_global];
        psi_t_in_Pframe = current_psi_t + rot_ang_P;
        
        addpoints(h_traj_relP, pos_T_in_Pframe(1), pos_T_in_Pframe(2));
        V_T_rel = get_arrow_vertices(pos_T_in_Pframe(1), pos_T_in_Pframe(2), psi_t_in_Pframe, arrow_scale);
        set(h_arrow_T_rel, 'Vertices', V_T_rel);

        % [Figure 5] Inertial Frame
        addpoints(h_traj_P_inertial, current_Xp, current_Yp);
        addpoints(h_traj_T_inertial, current_Xt, current_Yt);
        
        V_P_inert = get_arrow_vertices(current_Xp, current_Yp, current_psi_p, arrow_scale);
        V_T_inert = get_arrow_vertices(current_Xt, current_Yt, current_psi_t, arrow_scale);
        set(h_arrow_P_inertial, 'Vertices', V_P_inert);
        set(h_arrow_T_inertial, 'Vertices', V_T_inert);
        
        drawnow limitrate;
    end
    
    % =========================================================
    % [E] Stop Condition
    % =========================================================
    heading_diff_deg = abs(wrapTo180((current_psi_p - current_psi_t) * R2D));

    % % 1. [CPA 감지] 
    % if V_c < 0 && r < r_allow && heading_diff_deg < th_psi_deg
    %     hist_state = hist_state(:, 1:i); time = time(1:i);
    %     print_results_now(time(end), hist_state, V_p, V_t, R2D, '랑데뷰 성공 (CPA 감지, V_c < 0)');
    %     break;
    % end
    % 
    % % 2. [Relaxed Rendezvous]
    % if r < r_allow && heading_diff_deg < 1
    %     hist_state = hist_state(:, 1:i); time = time(1:i);
    %     print_results_now(time(end), hist_state, V_p, V_t, R2D, '완화 랑데뷰! (r < r_allow)');
    %     break;
    % end
    % 
    % % 3. [Pass Check]
    % if i > 1 && r > hist_state(1, i-1) && r < r_allow && heading_diff_deg < th_psi_deg
    %     hist_state = hist_state(:, 1:i-1); time = time(1:i-1);
    %     msg = sprintf('스치며 랑데뷰 성공 (최소거리=%.3fm)', hist_state(1, end));
    %     print_results_now(time(end), hist_state, V_p, V_t, R2D, msg);
    %     break;
    % end

    if i == num_steps
        msg = sprintf('Time Limit Reached (t=%.1fs, r=%.1fm)', time(end), r);
        print_results_now(time(end), hist_state, V_p, V_t, R2D, msg);
        warning('시뮬레이션 시간 종료! 랑데뷰/충돌 조건을 만족하지 못했습니다.');
    end
end

%% 6. [Figure 6, 7, 8, 9] 결과 분석 플롯 (Light Theme)

% 데이터 추출
r_data = hist_state(1,:);
lambda_data = hist_state(2,:) * R2D;
sigma_p_data = hist_state(5,:) * R2D;
sigma_t_data = hist_state(6,:) * R2D;
Vc_data = hist_state(7,:);
acc_p_data = hist_state(8,:);

% [수정] 배경 'w', 텍스트 'k'
figure(6); set(gcf, 'Position', [50 50 400 300], 'Color', 'w');
plot(time, r_data, 'r-', 'LineWidth', 2); grid on;
title('[Fig 6] Relative Distance', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); ylabel('Range [m]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

figure(7); set(gcf, 'Position', [460 50 400 300], 'Color', 'w');
plot(time, Vc_data, 'b-', 'LineWidth', 2); grid on;
title('[Fig 7] Closing Velocity (V_c)', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); ylabel('Speed [m/s]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

figure(8); set(gcf, 'Position', [870 50 400 300], 'Color', 'w');
plot(time, lambda_data, 'm-', 'LineWidth', 2); grid on;
title('[Fig 8] LOS Angle (\lambda)', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); ylabel('Angle [deg]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

figure(9); set(gcf, 'Position', [1280 50 400 300], 'Color', 'w');
plot(time, sigma_p_data, 'b-', 'LineWidth', 1.5); hold on;
plot(time, sigma_t_data, 'r--', 'LineWidth', 1.5); grid on;
title('[Fig 9] Lead Angles (\sigma)', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); ylabel('Angle [deg]', 'Color', 'k');
legend({'\sigma_p', '\sigma_t'}, 'TextColor', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

figure(10); set(gcf, 'Position', [50 400 400 300], 'Color', 'w');
plot(time, acc_p_data, 'r-', 'LineWidth', 2); grid on;
title('[Fig 10] Pursuer Acceleration Command', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); 
ylabel('Acc Command [m/s^2]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);
% [수정] 밝은 녹색 텍스트 -> 어두운 녹색('[0 0.5 0]')으로 변경
max_acc = max(abs(acc_p_data));
text(time(end)*0.5, max_acc*0.9, sprintf('Max: %.2f m/s^2', max_acc), ...
    'Color', [0 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');

figure(11); set(gcf, 'Position', [1280 400 400 300], 'Color', 'w');
% [수정] Cyan(하늘색) -> Blue(파란색 'b-')으로 변경 (흰 종이에 잘 보이게)
plot(r_data, acc_p_data, 'b-', 'LineWidth', 2); grid on;
set(gca, 'XDir', 'reverse'); 

title('[Fig 11] Acc vs Range (Range-to-Go)', 'Color', 'k');
xlabel('Relative Distance r [m]', 'Color', 'k'); 
ylabel('Acc Command [m/s^2]', 'Color', 'k'); 
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

max_acc = max(abs(acc_p_data));
text(time(end)*0.5, max_acc*0.9, sprintf('Max: %.2f m/s^2', max_acc), ...
    'Color', [0 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');

psi_p_data = hist_state(3,:) * R2D; % [추가] Pursuer Heading Data

figure(12); set(gcf, 'Position', [460 400 400 300], 'Color', 'w');
plot(time, psi_p_data, 'b-', 'LineWidth', 2); grid on;
title('[Fig 12] Pursuer Heading (\psi_p)', 'Color', 'k');
xlabel('Time [s]', 'Color', 'k'); 
ylabel('Angle [deg]', 'Color', 'k');
set(gca, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.3);

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
end