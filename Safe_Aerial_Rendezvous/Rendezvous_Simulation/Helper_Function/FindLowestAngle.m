%% FindLowestAngle.m
% 입력한 r에 대해 Reachable Region안에 속하는 점들 중 bearing angle이 가장 작은(Target의 Heading 방향과 가장 가까운) 점을 찾는 함수

% =========================================================
% 사용 예시 (이 부분을 메인 스크립트에서 실행)
% =========================================================
r_input = 100; 
min_theta = get_min_bearing_angle(r_input);

if isnan(min_theta)
    fprintf(' [결과] r = %.1f 인 지점에서는 영역에 포함되는 각도가 없는 것 같네.\n', r_input);
else
    fprintf(' [결과] r = %.1f 일 때 가장 작은 bearing angle은 약 %.1f도야.\n', r_input, min_theta);
end

function best_theta = get_min_bearing_angle(r_pt)
    % 1. 기본 파라미터 세팅 (메인 코드와 동일하게 유지)
    V = 20.0;                  
    g = 9.81;
    acc_limit = 1 * g;         
    
    scale = 1;
    r_f_max = 2.0;             
    r_target_radius = 2.0 * scale;     
    max_r_plot_limit = 20000;  
    epsilon = 1e-7;

    sigma_pc_deg_list = 0 : 0.05 : 90;
    num_sigma = length(sigma_pc_deg_list);
    
    % 2. 연산 속도 최적화: 각 sigma_pc 별 r_f_min 미리 계산
    valid_sigma_idx = false(1, num_sigma);
    r_f_min_list = zeros(1, num_sigma);
    
    % 최대값을 찾기 위한 배열 (속도를 위해 적당한 크기로 타협)
    sigma_t_calc = linspace(0, pi, 10000); 
    
    for i_sigma = 1:num_sigma
        sigma_pc_rad = deg2rad(sigma_pc_deg_list(i_sigma));
        if abs(cos(sigma_pc_rad)) < 1e-10
            continue;
        end
        
        y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* ...
                 (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ ...
                 (cos(sigma_pc_rad)^2 + epsilon);
             
        r_f_min = ((max(y_calc) * V^2) / (2 * acc_limit));
        
        if r_f_min <= r_f_max && r_f_min >= 0
            valid_sigma_idx(i_sigma) = true;
            r_f_min_list(i_sigma) = r_f_min;
        end
    end
    
    % 3. 0도(Target Heading)부터 양방향으로 각도 탐색 시작
    d_theta = 0.1; % 0.1도 간격으로 촘촘하게 탐색
    best_theta = NaN;
    
    for theta_abs = 0 : d_theta : 180
        % +방향과 -방향 번갈아가며 확인
        for sign_th = [1, -1]
            % 0도일 때는 두 번 검사할 필요 없으니 넘어가기
            if theta_abs == 0 && sign_th == -1
                continue;
            end
            
            theta_test_deg = theta_abs * sign_th;
            theta_pt_rad = deg2rad(theta_test_deg);
            
            % 방위각(Bearing Angle) 기준 좌표 역산
            x_pt = r_pt * sin(theta_pt_rad);
            y_pt = r_pt * cos(theta_pt_rad);
            sigma_t_pt = atan2(-x_pt, -y_pt);
            
            is_inside = false;
            
            % 미리 계산해둔 유효한 sigma_pc에 대해서만 영역 포함 여부 검사
            for i_sigma = find(valid_sigma_idx)
                sigma_pc_rad = deg2rad(sigma_pc_deg_list(i_sigma));
                r_f_min = r_f_min_list(i_sigma);
                
                if sigma_t_pt >= -sigma_pc_rad && sigma_t_pt <= (pi - sigma_pc_rad)
                    denominator = cos((sigma_t_pt + sigma_pc_rad) / 2)^2;
                    r_min_val = r_f_min * cos(sigma_pc_rad)^2 / denominator;
                    r_max_val = r_f_max * cos(sigma_pc_rad)^2 / denominator;
                    r_max_val = min(r_max_val, max_r_plot_limit);
            
                    if r_pt >= r_min_val && r_pt <= r_max_val && r_min_val >= r_target_radius
                        is_inside = true;
                        break; % 하나라도 겹치면 해당 각도는 유효함
                    end
                end
            end
            
            % 영역 안에 들어가는 첫 번째 각도가 0도와 가장 가까운 각도
            if is_inside
                best_theta = theta_test_deg;
                return;
            end
        end
    end
end