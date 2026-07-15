function draw_Accumulated_IntersectRegion(ax, V, acc_limit, r_f_max)
    % 기존 누적 영역 스크립트를 함수화하여 특정 axes에 그리도록 변환
    hold(ax, 'on');
    
    scale = 1;                 
    r_target_radius = 2.0 * scale;     
    max_r_plot_limit = 20000;  
    epsilon = 1e-7;
    
    sigma_pc_deg_list = 0 : 0.05 : 90;
    dark_green = [0.6, 0.6, 0.6];
    
    for i_sigma = 1:length(sigma_pc_deg_list)
        sigma_pc_deg = sigma_pc_deg_list(i_sigma);
        sigma_pc_rad = deg2rad(sigma_pc_deg);
        
        if abs(cos(sigma_pc_rad)) < 1e-10
            continue;
        end
        
        sigma_t_calc = linspace(0, pi, 50000);
        y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
        y_max_value = max(y_calc);
        r_f_min = ((y_max_value * V^2) / (2 * acc_limit));
        
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
            
            % 다각형(Polygon)을 덧그리기 (투명도 0.1 적용)[cite: 2]
            fill(ax, x_polygon, y_polygon, dark_green, 'EdgeColor', 'none', 'FaceAlpha', 0.1);
        end
    end
end