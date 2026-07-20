function draw_ReachableRegion_forMultipleplots(ax, V, acc_limit, r_f_max, xt, yt, psi_t)
    hold(ax, 'on');
         
    r_target_radius = 2.0;
    max_r_plot_limit = 20000;  
    epsilon = 1e-7;
    
    sigma_pc_deg_list = 0 : 0.05 : 90;
    region_color = [0.8, 0.9, 0.88];
    
    for i_sigma = 1:length(sigma_pc_deg_list)
        sigma_pc_deg = sigma_pc_deg_list(i_sigma);
        sigma_pc_rad = deg2rad(sigma_pc_deg);
        
        if abs(cos(sigma_pc_rad)) < 1e-10
            continue;
        end
        
        sigma_t_calc = linspace(0, pi, 50000);
        y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
        y_max_value = max(y_calc);
        r_f_min = ((y_max_value * V^2) / (2 * acc_limit*9.81));
        
        if r_f_min > r_f_max || r_f_min < 0
            continue;
        end
        
        sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);
        denominator = cos((sigma_t_traj + sigma_pc_rad) / 2).^2;
        r_min_traj = r_f_min * cos(sigma_pc_rad)^2 ./ denominator;
        r_max_traj = r_f_max * cos(sigma_pc_rad)^2 ./ denominator;
        r_max_traj = min(r_max_traj, max_r_plot_limit);
        
        valid_idx = isfinite(r_min_traj) & isfinite(r_max_traj) & (r_min_traj >= r_target_radius) & (r_max_traj >= r_min_traj);
        valid_number = find(valid_idx);
        
        if isempty(valid_number)
            continue;
        end
        
        separate_point = [1, find(diff(valid_number) > 1) + 1, length(valid_number) + 1];
        
        for i_segment = 1:length(separate_point) - 1
            segment_idx = valid_number(separate_point(i_segment) : separate_point(i_segment + 1) - 1);
            
            if length(segment_idx) < 2
                continue;
            end
            
            plot_angle = -pi/2 - sigma_t_traj(segment_idx);
            
            x_min = r_min_traj(segment_idx) .* cos(plot_angle);
            y_min = r_min_traj(segment_idx) .* sin(plot_angle);
            x_max = r_max_traj(segment_idx) .* cos(plot_angle);
            y_max = r_max_traj(segment_idx) .* sin(plot_angle);
            
            x_polygon = [x_min, fliplr(x_max)];
            y_polygon = [y_min, fliplr(y_max)];
            
            finite_polygon = isfinite(x_polygon) & isfinite(y_polygon);
            x_polygon = x_polygon(finite_polygon);
            y_polygon = y_polygon(finite_polygon);
            
            if length(x_polygon) < 3
                continue;
            end
            
            % 1. 회전 변환: 기준 방향(+y축, 즉 pi/2)에서 Target의 실제 Heading(psi_t)으로 회전
            theta = psi_t - pi/2;
            x_rot = x_polygon .* cos(theta) - y_polygon .* sin(theta);
            y_rot = x_polygon .* sin(theta) + y_polygon .* cos(theta);
            
            % 2. 평행 이동: Target의 실제 위치(xt, yt)로 이동
            x_final = x_rot + xt;
            y_final = y_rot + yt;
            
            % 3. 덧그리기
            fill(ax, x_final, y_final, region_color, 'EdgeColor', 'none', 'FaceAlpha', 0.05);
        end
    end 
end 