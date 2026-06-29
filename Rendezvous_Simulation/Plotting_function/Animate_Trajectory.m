function Animate_Trajectory(cfg, sim_out)
    R2D = 180/pi;
    D2R = pi/180;
    arrow_scale = 7;
    
    % 데이터 언패킹
    time = sim_out.time;
    hist_state = sim_out.hist_state;
    Xp_i = sim_out.init.Xp_i; Yp_i = sim_out.init.Yp_i;
    Xt_i = sim_out.init.Xt_i; Yt_i = sim_out.init.Yt_i;
    psi_p = sim_out.init.psi_p; psi_t = sim_out.init.psi_t;
    lambda_init = sim_out.init.lambda_init;
    r_i = sim_out.init.r_i; bearing_deg = sim_out.init.bearing_deg;
    
    bearing = bearing_deg * D2R; 
    
    V_p = sim_out.param.V_p; V_t = sim_out.param.V_t;

    lambda_deg_txt = lambda_init * R2D;
    psi_p_deg_txt = psi_p * R2D;
    psi_t_deg_txt = psi_t * R2D;
    sigma_p_deg_txt = (psi_p - lambda_init) * R2D;
    sigma_t_deg_txt = (psi_t - lambda_init) * R2D;

    % =========================================================
    % Figure 1: Global Inertial Frame (Initial)
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
    % Figure 2: Target Body Frame (Initial)
    % =========================================================
    figure(2); set(gcf, 'Position', [560 300 500 500], 'Theme', 'light');
    hold on; grid on; axis equal;
    title('[Fig 2] Initial Condition: Target Body Frame (Bearing)', 'Color', 'k');
    
    rot_angle = (pi/2) - psi_t;
    R_mat = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
    P_rel_vec = [Xp_i - Xt_i; Yp_i - Yt_i];
    P_body = R_mat * P_rel_vec;
    psi_p_body = psi_p + rot_angle;

    plot([0 0], [0 r_i*1.1], 'k:', 'LineWidth', 0.5); 
    plot([0 P_body(1)], [0 P_body(2)], 'Color', [0 0.5 0], 'LineStyle', '--', 'LineWidth', 1); 
    quiver(0, 0, 0, arrow_scale*5, 'Color', 'r', 'LineWidth', 3, 'MaxHeadSize',0.5, 'AutoScale','off'); 
    quiver(P_body(1), P_body(2), arrow_scale*5*cos(psi_p_body), arrow_scale*5*sin(psi_p_body), 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize',0.5, 'AutoScale','off'); 
    plot(0, 0, 'ro', 'MarkerFaceColor', 'r'); plot(P_body(1), P_body(2), 'bo', 'MarkerFaceColor', 'b');

    arc_r = r_i * 0.2;
    ang_start = pi/2; ang_end = pi/2 - bearing;
    t_arc = linspace(ang_start, ang_end, 30);
    plot(arc_r*cos(t_arc), arc_r*sin(t_arc), 'r-', 'LineWidth', 1.5);
    text(arc_r*0.7*cos(mean(t_arc)), arc_r*0.7*sin(mean(t_arc)), sprintf('Bearing %.1fdeg', bearing_deg), 'Color', 'r');

    % =========================================================
    % Figure 3, 4, 5: Animation Setup
    % =========================================================
    fig3 = figure('Position', [50 50 450 450], 'Name', '3. Target Centered', 'Theme', 'light'); grid on; axis equal; hold on;
    h_traj_relT = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_arrow_T_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r'); 
    h_arrow_P_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b'); 

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
        
        if mod(i, cfg.skip_frame) == 0 || i == 1
            dx_global = current_Xp - current_Xt;
            dy_global = current_Yp - current_Yt;
            
            % Fig 3
            rot_ang_T = (pi/2) - current_psi_t; 
            R_mat_T = [cos(rot_ang_T), -sin(rot_ang_T); sin(rot_ang_T), cos(rot_ang_T)];
            pos_P_in_Tframe = R_mat_T * [dx_global; dy_global]; 
            addpoints(h_traj_relT, pos_P_in_Tframe(1), pos_P_in_Tframe(2));
            set(h_arrow_P_rel, 'Vertices', get_arrow_vertices(pos_P_in_Tframe(1), pos_P_in_Tframe(2), current_psi_p + rot_ang_T, arrow_scale));
            set(h_arrow_T_fixed, 'Vertices', get_arrow_vertices(0, 0, pi/2, arrow_scale));

            % Fig 4
            rot_ang_P = (pi/2) - current_psi_p; 
            R_mat_P = [cos(rot_ang_P), -sin(rot_ang_P); sin(rot_ang_P), cos(rot_ang_P)];
            pos_T_in_Pframe = R_mat_P * [-dx_global; -dy_global];
            addpoints(h_traj_relP, pos_T_in_Pframe(1), pos_T_in_Pframe(2));
            set(h_arrow_T_rel, 'Vertices', get_arrow_vertices(pos_T_in_Pframe(1), pos_T_in_Pframe(2), current_psi_t + rot_ang_P, arrow_scale));
            set(h_arrow_P_fixed, 'Vertices', get_arrow_vertices(0, 0, pi/2, arrow_scale));

            % Fig 5
            addpoints(h_traj_P_inertial, current_Xp, current_Yp);
            addpoints(h_traj_T_inertial, current_Xt, current_Yt);
            set(h_arrow_P_inertial, 'Vertices', get_arrow_vertices(current_Xp, current_Yp, current_psi_p, arrow_scale));
            set(h_arrow_T_inertial, 'Vertices', get_arrow_vertices(current_Xt, current_Yt, current_psi_t, arrow_scale));
            
            drawnow;
            
            % --- 이 부분이 핵심! 화면에 그릴 시간을 벌어주는 거야 ---
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

function V_rot = get_arrow_vertices(x, y, psi, scale)
    V_base = [ 1.5,  0.0; -1.0,  0.8; -0.5,  0.0; -1.0, -0.8] * scale;
    R_mat = [cos(psi), -sin(psi); sin(psi),  cos(psi)];
    V_rot = (R_mat * V_base')'; 
    V_rot(:,1) = V_rot(:,1) + x; 
    V_rot(:,2) = V_rot(:,2) + y; 
end