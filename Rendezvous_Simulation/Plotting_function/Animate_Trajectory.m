function Animate_Trajectory(cfg, sim_out)
    R2D = 180/pi;
    D2R = pi/180;
    arrow_scale = 7;
    
    % 데이터 언패킹[cite: 6]
    time = sim_out.time;
    hist_state = sim_out.hist_state;
    Xp_i = sim_out.init.Xp_i; Yp_i = sim_out.init.Yp_i;
    Xt_i = sim_out.init.Xt_i; Yt_i = sim_out.init.Yt_i;
    psi_p = sim_out.init.psi_p; psi_t = sim_out.init.psi_t;
    lambda_init = sim_out.init.lambda_init;
    r_i = sim_out.init.r_i; bearing_deg = sim_out.init.bearing_deg;
    
    bearing = bearing_deg * D2R; 
    
    V_p = sim_out.param.V_p; V_t = sim_out.param.V_t;

    % cfg 파라미터에서 Region 계산용 변수 가져오기 (없으면 기본값 세팅)[cite: 6]
    if isfield(cfg, 'limit_acc'), acc_limit = cfg.limit_acc; else acc_limit = 9.81; end
    if isfield(cfg, 'r_allow'), r_f_max = cfg.r_allow; else r_f_max = 2.0; end
    
    lambda_deg_txt = lambda_init * R2D;
    psi_p_deg_txt = psi_p * R2D;
    psi_t_deg_txt = psi_t * R2D;
    
    initial_sigma_p = psi_p - lambda_init;
    sigma_p_deg_txt = initial_sigma_p * R2D;
    sigma_t_deg_txt = (psi_t - lambda_init) * R2D;

    % =========================================================
    % Figure 1: Global Inertial Frame (Initial)[cite: 6]
    % =========================================================
    figure(1); set(gcf, 'Position', [50 300 500 500], 'Theme', 'light');
    hold on; grid on; axis equal;
    title('[Fig 1] Initial Condition: Global Inertial Frame', 'Color', 'k');
    plot([Xp_i Xt_i], [Yp_i Yt_i], 'k--', 'LineWidth', 1); 
    quiver(Xp_i, Yp_i, arrow_scale*5*cos(psi_p), arrow_scale*5*sin(psi_p), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    quiver(Xt_i, Yt_i, arrow_scale*5*cos(psi_t), arrow_scale*5*sin(psi_t), 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    plot(Xp_i, Yp_i, 'bo', 'MarkerFaceColor', 'b'); plot(Xt_i, Yt_i, 'ro', 'MarkerFaceColor', 'r');

    info_p = sprintf('  Pursuer\n  \\psi_p: %.1f\\circ\n  \\sigma_p: %.1f\\circ', psi_p_deg_txt, sigma_p_deg_txt);
    text(Xp_i, Yp_i, info_p, 'Color', 'b', 'VerticalAlignment', 'top', 'FontWeight', 'bold'); 

    info_t = sprintf('  Target\n  \\psi_t: %.1f\\circ\n  \\sigma_t: %.1f\\circ', psi_t_deg_txt, sigma_t_deg_txt);
    text(Xt_i, Yt_i, info_t, 'Color', [0.7 0 0.7], 'VerticalAlignment', 'top', 'FontWeight', 'bold'); 

    mid_x = (Xp_i + Xt_i) / 2; mid_y = (Yp_i + Yt_i) / 2;
    text(mid_x, mid_y, sprintf('  \\lambda: %.1f\\circ', lambda_deg_txt), ...
        'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
    legend('LOS', 'Pursuer Vel', 'Target Vel', 'Location', 'best');

    % =========================================================
    % Figure 2: Target Body Frame (Initial & Static Regions)[cite: 6]
    % =========================================================
    figure(2); set(gcf, 'Position', [560 300 500 500], 'Theme', 'light');
    hold on; grid on; axis equal; xlim([-r_i*1.8, r_i*0.2]); ylim([-r_i*0.1, r_i*1.9]);
    title('[Fig 2] Initial Condition: Target Body Frame', 'Color', 'k');
    
    rot_angle = (pi/2) - psi_t;
    R_mat = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
    P_rel_vec = [Xp_i - Xt_i; Yp_i - Yt_i];
    P_body = R_mat * P_rel_vec;
    psi_p_body = psi_p + rot_angle;

    epsilon = 1e-7;
    r_target_radius = 2.0;
    
    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(initial_sigma_p)) .* (1 + cos(sigma_t_calc + initial_sigma_p)) ./ (cos(initial_sigma_p)^2 + epsilon);
    [y_max, ~] = max(y_calc);
    r_f_min = (y_max * V_p^2) / (2 * acc_limit);
    
    sigma_t_traj_full = linspace(-initial_sigma_p, pi - initial_sigma_p - 1e-5, 500);
    r_min_traj_full = r_f_min * (cos(initial_sigma_p))^2 ./ cos((sigma_t_traj_full + initial_sigma_p)/2).^2;
    r_max_traj_full = r_f_max * (cos(initial_sigma_p))^2 ./ cos((sigma_t_traj_full + initial_sigma_p)/2).^2;
    
    valid_idx_dpg = (r_max_traj_full >= r_target_radius);
    if any(valid_idx_dpg)
        plot_angle_dpg = -pi/2 - sigma_t_traj_full(valid_idx_dpg);
        x_dpg_min = r_target_radius .* cos(plot_angle_dpg);
        y_dpg_min = r_target_radius .* sin(plot_angle_dpg);
        x_dpg_max = r_max_traj_full(valid_idx_dpg) .* cos(plot_angle_dpg);
        y_dpg_max = r_max_traj_full(valid_idx_dpg) .* sin(plot_angle_dpg);
        
        X_dpg = [x_dpg_min, fliplr(x_dpg_max)];
        Y_dpg = [y_dpg_min, fliplr(y_dpg_max)];
        patch(X_dpg, Y_dpg, [0.8 0.7 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'DPG Reachable Region');
    end

    valid_idx_int = (r_min_traj_full >= r_target_radius) & (r_max_traj_full <= 2000); 
    if r_f_min <= r_f_max && any(valid_idx_int)
        plot_angle_int = -pi/2 - sigma_t_traj_full(valid_idx_int);
        x_min = r_min_traj_full(valid_idx_int) .* cos(plot_angle_int);
        y_min = r_min_traj_full(valid_idx_int) .* sin(plot_angle_int);
        x_max = r_max_traj_full(valid_idx_int) .* cos(plot_angle_int);
        y_max = r_max_traj_full(valid_idx_int) .* sin(plot_angle_int);
        
        X_fill = [x_min, fliplr(x_max)];
        Y_fill = [y_min, fliplr(y_max)];
        fill(X_fill, Y_fill, [0 0.8 0], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Intersect Region');
    end
    
    % --- [추가] 초기 +g Limit 영역 그리기 ---
    sigma_t_g_init = linspace(0, pi, 300);
    r_glimit_init = (V_p^2 / acc_limit) * (sin(sigma_t_g_init) - sin(initial_sigma_p));
    valid_g_init = r_glimit_init > 0;
    if any(valid_g_init)
        plot_angle_g_init = -pi/2 - sigma_t_g_init(valid_g_init);
        x_g_init = r_glimit_init(valid_g_init) .* cos(plot_angle_g_init);
        y_g_init = r_glimit_init(valid_g_init) .* sin(plot_angle_g_init);
        plot(x_g_init, y_g_init, 'r', 'LineWidth', 1.5, 'DisplayName', '+g Limit');
    end

    plot([0 0], [0 r_i*1.1], 'k:', 'LineWidth', 0.5, 'HandleVisibility','off'); 
    plot([0 P_body(1)], [0 P_body(2)], 'Color', [0 0.5 0], 'LineStyle', '--', 'LineWidth', 1, 'HandleVisibility','off'); 
    quiver(0, 0, 0, arrow_scale*5, 'Color', 'r', 'LineWidth', 3, 'MaxHeadSize',0.5, 'AutoScale','off', 'DisplayName','Target Vel'); 
    quiver(P_body(1), P_body(2), arrow_scale*5*cos(psi_p_body), arrow_scale*5*sin(psi_p_body), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize',0.5, 'AutoScale','off', 'DisplayName','Pursuer Vel'); 
    plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility','off'); 
    plot(P_body(1), P_body(2), 'bo', 'MarkerFaceColor', 'b', 'HandleVisibility','off');

    arc_r = r_i * 0.2;
    ang_start = pi/2; ang_end = pi/2 - bearing;
    t_arc = linspace(ang_start, ang_end, 30);
    plot(arc_r*cos(t_arc), arc_r*sin(t_arc), 'r-', 'LineWidth', 1.5, 'HandleVisibility','off');
    text(arc_r*0.7*cos(mean(t_arc)), arc_r*0.7*sin(mean(t_arc)), sprintf('Bearing %.1fdeg', bearing_deg), 'Color', 'r');
    legend('Location', 'best');

    % =========================================================
    % Figure 3, 4, 5: Animation Setup[cite: 6]
    % =========================================================
    fig3 = figure('Position', [50 50 450 450], 'Name', '3. Target Centered', 'Theme', 'light'); grid on; axis equal; hold on;
    % [추가] Figure 3용 Region Patch (초기엔 빈 데이터로 세팅)
    h_dpg_region_fig3 = patch(nan, nan, [0.8 0.7 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    h_int_region_fig3 = patch(nan, nan, [0 0.8 0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    
    % ---------------------------------------------------------
    % [추가] RDPG_VT 모드일 때 가상 타겟(Virtual Target) 고정 표시
    % ---------------------------------------------------------
    if isfield(cfg, 'GUIDANCE_MODE') && strcmp(cfg.GUIDANCE_MODE, 'RDPG_VT')
        % Target Centered Frame에서는 Target이 항상 원점(0,0)에서 +y 방향을 바라봄.
        % 따라서 Virtual Target의 위치는 좌표계 회전 없이 바로 계산 가능해.
        vt_x_fig3 = cfg.r_vt * sin(cfg.theta_vt_rad);
        vt_y_fig3 = cfg.r_vt * cos(cfg.theta_vt_rad);
        
        % 연보라색 별(p) 모양으로 흐릿하게(MarkerFaceAlpha) 표시
        scatter(vt_x_fig3, vt_y_fig3, 200, 'p', ...
            'MarkerFaceColor', [0.7 0.5 0.9], ...
            'MarkerEdgeColor', 'none', ...
            'MarkerFaceAlpha', 0.5);
            
        text(vt_x_fig3, vt_y_fig3 - r_i*0.05, 'VT', ...
            'Color', [0.6 0.4 0.8], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    % ---------------------------------------------------------
    h_glimit_fig3 = plot(nan, nan, 'r', 'LineWidth', 1.5); % [추가] 실시간 +g Limit
    
    h_traj_relT = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_arrow_T_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r'); 
    h_arrow_P_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b'); 
    xlim([-r_i, r_i * 0.01]); ylim([-r_i*0.5, r_i*0.5]);

    fig4 = figure('Position', [510 50 450 450], 'Name', '4. Pursuer Centered', 'Theme', 'light'); grid on; axis equal; hold on;
    h_traj_relP = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
    h_arrow_P_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b');
    h_arrow_T_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r');

    fig5 = figure('Position', [970 50 450 450], 'Name', '5. Inertial', 'Theme', 'light'); grid on; axis equal; hold on;
    h_traj_P_inertial = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_traj_T_inertial = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
    h_arrow_P_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b');
    h_arrow_T_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r');

    if cfg.auto_save == 1
        if ~exist(cfg.save_dir, 'dir'), mkdir(cfg.save_dir); end
        vidObj3 = VideoWriter(fullfile(cfg.save_dir, 'Figure_3.mp4'), 'MPEG-4'); open(vidObj3);
        vidObj4 = VideoWriter(fullfile(cfg.save_dir, 'Figure_4.mp4'), 'MPEG-4'); open(vidObj4);
        vidObj5 = VideoWriter(fullfile(cfg.save_dir, 'Figure_5.mp4'), 'MPEG-4'); open(vidObj5);
    end

    current_Xp = Xp_i; current_Yp = Yp_i;
    current_Xt = Xt_i; current_Yt = Yt_i;
    
    for i = 1:length(time)
        current_psi_p = hist_state(3, i);
        current_psi_t = hist_state(4, i);
        current_sigma_p = hist_state(5, i);
        
        if mod(i, cfg.skip_frame) == 0 || i == 1
            dx_global = current_Xp - current_Xt;
            dy_global = current_Yp - current_Yt;
            
            % --- Fig 3: Target Centered Frame & 동적 Region 업데이트 ---
            rot_ang_T = (pi/2) - current_psi_t; 
            R_mat_T = [cos(rot_ang_T), -sin(rot_ang_T); sin(rot_ang_T), cos(rot_ang_T)];
            pos_P_in_Tframe = R_mat_T * [dx_global; dy_global]; 
            
            addpoints(h_traj_relT, pos_P_in_Tframe(1), pos_P_in_Tframe(2));
            set(h_arrow_P_rel, 'Vertices', get_arrow_vertices(pos_P_in_Tframe(1), pos_P_in_Tframe(2), current_psi_p + rot_ang_T, arrow_scale));
            set(h_arrow_T_fixed, 'Vertices', get_arrow_vertices(0, 0, pi/2, arrow_scale));

            % 현재 리드각 기준으로 Boundary 계산해서 실시간으로 덮어씌우기
            [X_int, Y_int, X_dpg, Y_dpg, X_g, Y_g] = get_analytic_regions(current_sigma_p, V_p, acc_limit, r_f_max);
            
            if isempty(X_dpg)
                set(h_dpg_region_fig3, 'XData', nan, 'YData', nan);
            else
                set(h_dpg_region_fig3, 'XData', X_dpg, 'YData', Y_dpg);
            end
            
            if isempty(X_int)
                set(h_int_region_fig3, 'XData', nan, 'YData', nan);
            else
                set(h_int_region_fig3, 'XData', X_int, 'YData', Y_int);
            end

            % [추가] 실시간 +g Limit 업데이트
            if isempty(X_g)
                set(h_glimit_fig3, 'XData', nan, 'YData', nan);
            else
                set(h_glimit_fig3, 'XData', X_g, 'YData', Y_g);
            end

            % --- Fig 4 ---
            rot_ang_P = (pi/2) - current_psi_p; 
            R_mat_P = [cos(rot_ang_P), -sin(rot_ang_P); sin(rot_ang_P), cos(rot_ang_P)];
            pos_T_in_Pframe = R_mat_P * [-dx_global; -dy_global];
            addpoints(h_traj_relP, pos_T_in_Pframe(1), pos_T_in_Pframe(2));
            set(h_arrow_T_rel, 'Vertices', get_arrow_vertices(pos_T_in_Pframe(1), pos_T_in_Pframe(2), current_psi_t + rot_ang_P, arrow_scale));
            set(h_arrow_P_fixed, 'Vertices', get_arrow_vertices(0, 0, pi/2, arrow_scale));

            % --- Fig 5 ---
            addpoints(h_traj_P_inertial, current_Xp, current_Yp);
            addpoints(h_traj_T_inertial, current_Xt, current_Yt);
            set(h_arrow_P_inertial, 'Vertices', get_arrow_vertices(current_Xp, current_Yp, current_psi_p, arrow_scale));
            set(h_arrow_T_inertial, 'Vertices', get_arrow_vertices(current_Xt, current_Yt, current_psi_t, arrow_scale));
            
            drawnow;
            
            pause(0.005); 
            
            if cfg.auto_save == 1
                writeVideo(vidObj3, getframe(fig3));
                writeVideo(vidObj4, getframe(fig4));
                writeVideo(vidObj5, getframe(fig5));
            end
        end
        
        if i < length(time)
            dt = time(i+1) - time(i);
            current_Xp = current_Xp + V_p * cos(current_psi_p) * dt;
            current_Yp = current_Yp + V_p * sin(current_psi_p) * dt;
            current_Xt = current_Xt + V_t * cos(current_psi_t) * dt;
            current_Yt = current_Yt + V_t * sin(current_psi_t) * dt;
        end
    end
    
    if cfg.auto_save == 1
        close(vidObj3); close(vidObj4); close(vidObj5);
    end
end

%% --- 헬퍼 함수들 ---
function V_rot = get_arrow_vertices(x, y, psi, scale)
    V_base = [ 1.5,  0.0; -1.0,  0.8; -0.5,  0.0; -1.0, -0.8] * scale;
    R_mat = [cos(psi), -sin(psi); sin(psi),  cos(psi)];
    V_rot = (R_mat * V_base')'; 
    V_rot(:,1) = V_rot(:,1) + x; 
    V_rot(:,2) = V_rot(:,2) + y; 
end

function [X_int, Y_int, X_dpg, Y_dpg, X_glimit, Y_glimit] = get_analytic_regions(sigma_p_rad, V, acc_limit, r_f_max)
    % Meshgrid 없이 수식만으로 Region 폴리곤 영역 좌표를 뽑아내는 함수
    epsilon = 1e-7;
    r_target_radius = 2.0;
    
    sigma_t_calc = linspace(0, pi, 500);
    y_calc = (sin(sigma_t_calc) - sin(sigma_p_rad)) .* (1 + cos(sigma_t_calc + sigma_p_rad)) ./ (cos(sigma_p_rad)^2 + epsilon);
    y_max = max(y_calc);
    r_f_min = (y_max * V^2) / (2 * acc_limit);
    
    sigma_t_traj = linspace(-sigma_p_rad, pi - sigma_p_rad - 1e-5, 200);
    plot_angle_traj = -pi/2 - sigma_t_traj;
    
    r_max_traj = r_f_max * (cos(sigma_p_rad))^2 ./ cos((sigma_t_traj + sigma_p_rad)/2).^2;
    
    valid_idx_dpg = (r_max_traj >= r_target_radius);
    if any(valid_idx_dpg)
        x_dpg_min = r_target_radius .* cos(plot_angle_traj(valid_idx_dpg));
        y_dpg_min = r_target_radius .* sin(plot_angle_traj(valid_idx_dpg));
        x_dpg_max = r_max_traj(valid_idx_dpg) .* cos(plot_angle_traj(valid_idx_dpg));
        y_dpg_max = r_max_traj(valid_idx_dpg) .* sin(plot_angle_traj(valid_idx_dpg));
        
        X_dpg = [x_dpg_min, fliplr(x_dpg_max)];
        Y_dpg = [y_dpg_min, fliplr(y_dpg_max)];
    else
        X_dpg = []; Y_dpg = [];
    end

    if r_f_min <= r_f_max
        r_min_traj = r_f_min * (cos(sigma_p_rad))^2 ./ cos((sigma_t_traj + sigma_p_rad)/2).^2;
        valid_idx_int = (r_min_traj >= r_target_radius) & (r_max_traj <= 2000); 
        
        if any(valid_idx_int)
            x_min = r_min_traj(valid_idx_int) .* cos(plot_angle_traj(valid_idx_int));
            y_min = r_min_traj(valid_idx_int) .* sin(plot_angle_traj(valid_idx_int));
            x_max = r_max_traj(valid_idx_int) .* cos(plot_angle_traj(valid_idx_int));
            y_max_curve = r_max_traj(valid_idx_int) .* sin(plot_angle_traj(valid_idx_int));
            
            X_int = [x_min, fliplr(x_max)];
            Y_int = [y_min, fliplr(y_max_curve)];
        else
            X_int = []; Y_int = [];
        end
    else
        X_int = []; Y_int = [];
    end

    % --- [추가] +g Limit Curve (빨간선) ---
    sigma_t_g = linspace(0, pi, 300);
    r_g = (V^2 / acc_limit) * (sin(sigma_t_g) - sin(sigma_p_rad));
    valid_g = r_g > 0;
    if any(valid_g)
        plot_angle_g = -pi/2 - sigma_t_g(valid_g);
        X_glimit = r_g(valid_g) .* cos(plot_angle_g);
        Y_glimit = r_g(valid_g) .* sin(plot_angle_g);
    else
        X_glimit = []; Y_glimit = [];
    end
end