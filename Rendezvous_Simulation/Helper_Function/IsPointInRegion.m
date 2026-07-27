function is_inside = IsPointInRegion(r_pt, theta_pt_deg, V, acc_limit, r_f_max)
    % 입력한 점이 Reachable Region 안에 있는지 판별하는 함수

    epsilon = 1e-7;
    sigma_pc_deg_list = 0 : 0.05 : 90;
    num_sigma = length(sigma_pc_deg_list);

    % Input Point (Bearing Angle)
    theta_pt_rad = deg2rad(theta_pt_deg);
    
    % Target heading(+y축)을 기준으로 CW(+), CCW(-)
    x_pt = r_pt * sin(theta_pt_rad);
    y_pt = r_pt * cos(theta_pt_rad);
    
    % 역산
    sigma_t_pt = atan2(-x_pt, -y_pt);
    is_inside = false;
    
    for i_sigma = 1:num_sigma
        sigma_pc_deg = sigma_pc_deg_list(i_sigma);
        sigma_pc_rad = deg2rad(sigma_pc_deg);
    
        if abs(cos(sigma_pc_rad)) < 1e-10
            continue;
        end
    
        % r_f_min 계산
        sigma_t_calc = linspace(0, pi, 50000);
        y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
        r_f_min = ((max(y_calc) * V^2) / (2 * acc_limit));
    
        if r_f_min > r_f_max || r_f_min < 0
            continue;
        end
    
        % 해당 sigma_pc에서 점의 각도가 유효한지 확인
        if sigma_t_pt >= -sigma_pc_rad && sigma_t_pt <= (pi - sigma_pc_rad)
            
            % 해당 각도에서의 r_min과 r_max 경계 계산
            denominator = cos((sigma_t_pt + sigma_pc_rad) / 2)^2;
            r_min_val = r_f_min * cos(sigma_pc_rad)^2 / denominator;
            r_max_val = r_f_max * cos(sigma_pc_rad)^2 / denominator;
            r_max_val = min(r_max_val, 20000);
    
            % 점의 거리(r)가 해당 각도에서의 경계 안쪽에 있는지 판별
            if r_pt >= r_min_val && r_pt <= r_max_val && r_min_val >= r_f_max
                is_inside = true;
                break; 
            end
        end
    end
end