function Plot_MaxAccPoint(cfg, sim_out, theory)
%% Plot_MaxAccPoint.m
%% 0. Parameter Setting
    V = cfg.V_p;
    g = 9.81;
    acc_limit = 1 * g; % 검증용 맵 배경은 1g 기준으로 고정
    
    sigma_pc_deg = cfg.target_lead_angle_deg;
    sigma_pc_rad = deg2rad(sigma_pc_deg);
    r_f_max = cfg.r_allow;
    
    % sim_out에서 데이터 언패킹
    r_data = sim_out.hist_state(1, :);
    sigma_t_data = sim_out.hist_state(6, :); 
    acc_data = sim_out.hist_state(8, :);
    
    [~, max_idx] = max(abs(acc_data));
    r_sim_max = r_data(max_idx);
    sigmat_sim_max = sigma_t_data(max_idx);
    
    r_final = r_data(end);
    sigmat_final = sigma_t_data(end);
    
%% 1. Plot Contour and Max Acceleration Region
    r_target_radius = 2.0; % 기준 반경 설정
    
    % 2m부터 시작해서 앞쪽을 촘촘하게 나누기
    r_range = [linspace(r_target_radius, 50, 1500), linspace(50.5, 1000, 500)]; 
    theta_range = linspace(deg2rad(90), deg2rad(270), 400);
    [R_grid, Theta_grid] = meshgrid(r_range, theta_range);

    Px_grid = R_grid .* cos(Theta_grid);
    Py_grid = R_grid .* sin(Theta_grid);

    Lam_grid = Theta_grid + pi;        
    Sigma_t_grid = pi/2 - Lam_grid;  % 타겟 헤딩 psi_t = pi/2 가정
    
    Acc_cmd_grid = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(sigma_pc_rad));
    
    % 2m 미만 영역은 NaN 처리해서 배경 비우기
    Acc_cmd_grid(R_grid < r_target_radius) = NaN;
    
    fig = figure('Name', 'Max Acc Point Validation', 'Theme', 'light', 'Position', [100, 100, 800, 700]);
    ax = gca; 
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    % Colormap (Safe / +g Limit 전용)
    cmap = zeros(256, 3);
    cmap(1:128, :) = repmat([0.7 0.8 1.0], 128, 1);    % Safe Region (Blue)
    cmap(129:256, :) = repmat([1.0 0.7 0.7], 128, 1);  % +g Region (Red)
    
    contourf(Px_grid, Py_grid, Acc_cmd_grid, [-100*acc_limit, acc_limit, 100*acc_limit], 'LineColor', 'none');
    colormap(ax, cmap); clim(ax, [-acc_limit, 3*acc_limit]); 
    
    % +g Limit 선 그리기
    contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    
    % DPG Reachable Region 덧그리기
    if exist('draw_DPG_Region', 'file')
        draw_DPG_Region(ax, sigma_pc_rad, r_f_max);
    end
    
%% 2. Calculate R_f min and Intersect Region
    epsilon = 1e-7;
    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
    [y_max, ~] = max(y_calc);
    r_f_min = (y_max * V^2) / (2 * acc_limit);
    
    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 500);
    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    r_max_traj = r_f_max * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    
    max_r_plot_limit = 1000;
    
    valid_idx = (r_min_traj >= r_target_radius) & (r_max_traj <= max_r_plot_limit);
    
    r_min_traj = r_min_traj(valid_idx);
    r_max_traj = r_max_traj(valid_idx);
    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);
    
    x_min = r_min_traj .* cos(plot_angle_traj);
    y_min = r_min_traj .* sin(plot_angle_traj);
    x_max = r_max_traj .* cos(plot_angle_traj);
    y_max_curve = r_max_traj .* sin(plot_angle_traj);
    
    if r_f_min <= r_f_max && any(valid_idx)
        X_fill = [x_min, fliplr(x_max)];
        Y_fill = [y_min, fliplr(y_max_curve)];
        fill(X_fill, Y_fill, [0 0.8 0], 'EdgeColor', 'none', 'FaceAlpha', 1); 
        plot(x_min, y_min, 'g-', 'LineWidth', 2);
    end
    
%% 3. Mark Trajectory and Arrow
    % 밑단 가운데를 살짝 위로(-0.2) 파서 화살촉(Arrowhead) 느낌을 줬어.
    arrow_x = [0, -0.5, 0, 0.5];
    arrow_y = [1, -1, -0.5, -1];
    fill(ax, arrow_x, arrow_y, 'r', 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % Pursuer 전체 궤적 그리기 (파란색 점선)
    ang_traj = -pi/2 - sigma_t_data;
    x_traj = r_data .* cos(ang_traj);
    y_traj = r_data .* sin(ang_traj);
    h_traj = plot(x_traj, y_traj, 'b--', 'LineWidth', 2);

    % 1) Predicted Max Acc Point (이론값) -> 빨간 별
    r_pred = theory.r_star;
    ang_pred = -pi/2 - deg2rad(theory.sigma_t_star_deg);
    x_pred = r_pred * cos(ang_pred);
    y_pred = r_pred * sin(ang_pred);
    
    % 2) Simulated Max Acc Point (실측 피크) -> 파란 별
    ang_sim = -pi/2 - sigmat_sim_max;
    x_sim = r_sim_max * cos(ang_sim);
    y_sim = r_sim_max * sin(ang_sim);
    
    % 3) Final Point (시뮬레이션 종단점) -> 보라색 별(Purple)
    ang_final = -pi/2 - sigmat_final;
    x_final = r_final * cos(ang_final);
    y_final = r_final * sin(ang_final);
    
    % Target 중심(0,0)에서 최종 보라색 별까지 검은색 점선 긋기
    h_sim_rf = plot([0, x_final], [0, y_final], 'k--', 'LineWidth', 1.8);
    
    % 마커들 플롯 (궤적 위에 확실히 보이게)
    h_pred = plot(x_pred, y_pred, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'LineWidth', 1);
    h_sim = plot(x_sim, y_sim, 'bp', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'LineWidth', 1);
    h_final = plot(x_final, y_final, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.5 0 0.5], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    
%% 4. Axis and Legend Setting
    title(sprintf('Max Acc Validation: \\sigma_{p0} = %.1f^\\circ', sigma_pc_deg), 'FontSize', 14, 'FontWeight', 'bold');
    xlim([-550, 50]); ylim([-300, 300]);
    xlabel('x (m)', 'FontSize', 12); 
    ylabel('y (m)', 'FontSize', 12);
    
    % 범례 아이템 정리
    h_red   = plot(nan, nan, 'r', 'LineWidth', 1.5);
    h_safe  = patch(nan, nan, [0.7 0.8 1.0], 'EdgeColor', 'none'); 
    h_inter = patch(nan, nan, [0 0.8 0], 'EdgeColor', 'none'); 
    h_reach = patch(nan, nan, [0.8 0.7 0], 'EdgeColor', 'none'); 
    
    legend([h_red, h_safe, h_inter, h_reach, h_traj, h_pred, h_sim, h_sim_rf, h_final], ...
        {'+g Limit', 'Safe Region', 'Intersect Region', 'DPG Reachable Region', ...
         'Pursuer Trajectory', 'Predicted Max Acc Point', 'Simulated Max Acc Point', ...
         'Simulated R_f Line', 'Final Point (Purple Star)'}, ...
        'Location', 'northwest', 'FontSize', 10);
        
%% 5. Auto Save Images
    if cfg.auto_save == 1
        if ~exist(cfg.save_dir, 'dir'), mkdir(cfg.save_dir); end
        filename = sprintf('Plot_MaxAccPoint_%.1fdeg.png', sigma_pc_deg);
        print(fig, fullfile(cfg.save_dir, filename), '-dpng', '-r250');
        fprintf('>>> [저장 완료] %s <<<\n', filename);
    end
end