classdef RDPG_FRS < handle
    properties
        k, max_acc, r_f_max, dt
        sigma_ref_prev
        sigma_pref          % 62.6도와 같은 선호 각도[rad]
        t_for               % Fail-Safe 탐색을 위한 전방향 시뮬레이션 시간
        sigma_FRS_list      % Fail-Safe 탐색을 위한 sigma_pc 각도 리스트[deg]
        rate_limit          % Rate Limit (rad/s)
        is_latched          % sigma_pc 확정    
    end
    
    methods
        function obj = RDPG_FRS(k_gain, limit_G, r_f_max, dt, init_sigma_rad, sigma_pref, t_for, sigma_FRS_list, rate_limit)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; 
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
            obj.sigma_pref = sigma_pref;
            obj.t_for = t_for; 
            obj.sigma_FRS_list = sigma_FRS_list; 
            obj.rate_limit = rate_limit; % Rate limit 초기화
            obj.is_latched = false(1, length(sigma_FRS_list));
        end

        function [x_traj, y_traj] = get_Boundary(obj, sigma_pref, gamma_t)
            % 주어진 sigma_pref와 gamma_t로 Boundary line의 점들 생성 및 저장   
            r_f = obj.r_f_max;
            max_r_plot_limit = 20000;  

            sigma_t_traj = linspace(-sigma_pref, pi - sigma_pref - 1e-5, 10000);
            denominator = cos((sigma_t_traj + sigma_pref) / 2).^2;
            r_traj = r_f * cos(sigma_pref)^2 ./ denominator;
            r_traj = min(r_traj, max_r_plot_limit);

            valid_idx = isfinite(r_traj) & (r_traj >= 100);
            plot_sigma_t = sigma_t_traj(valid_idx);
            plot_r = r_traj(valid_idx);

            x_traj = [];
            y_traj = [];
            
            if ~isempty(plot_r)
                % Target의 실제 헤딩(gamma_t)을 반영하도록 수정
                plot_angle = gamma_t + pi - plot_sigma_t;
                x_traj = plot_r .* cos(plot_angle);
                y_traj = plot_r .* sin(plot_angle);
            end
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag, point_value] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t, V_t, gamma_t, x_curr, y_curr)
            
            % point_value 초기화 (Fail-Safe 모드가 아닐 때 에러 방지용)
            num_angles = length(obj.sigma_FRS_list);
            point_value = zeros(num_angles, 4);

            % =======================================================
            % Dynamic Epsilon 스케줄링 로직
            % =======================================================
            eps_max = 0.001;
            eps_min = 1e-7;
            r_fade_start = 200.0; 
            r_fade_end = 100.0;    
            if r >= r_fade_start
                dynamic_eps = eps_max;
            elseif r > r_fade_end
                alpha = (r - r_fade_end) / (r_fade_start - r_fade_end);
                dynamic_eps = eps_min + alpha * (eps_max - eps_min);
            else
                dynamic_eps = eps_min;
            end
            % =======================================================

            % -----------------------------------------------------------
            % [1단계] r_f_min 계산하여 현재 Pursuer가 Reachable Region 내에 있는지 확인
            % -----------------------------------------------------------
            sigma_t_calc = linspace(0, pi, 1000); 
            
            y_calc = (sin(sigma_t_calc) - sin(sigma_p)) .* (1 + cos(sigma_t_calc + sigma_p)) ./ (cos(sigma_p)^2 + dynamic_eps);
            y_max = max(y_calc);
            
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            denom_contour = cos((sigma_t + sigma_p)/2)^2 + dynamic_eps;
            r_contour_min = r_f_min * (cos(sigma_p))^2 / denom_contour;
            r_contour_max = obj.r_f_max * (cos(sigma_p))^2 / denom_contour;

            % -----------------------------------------------------------
            % [2단계] Safe / Unsafe 판별
            % -----------------------------------------------------------
            if r >= r_contour_min && r <= r_contour_max
                sigma_pc = sigma_p;
                mode_flag = 0; 
                obj.is_latched(:) = false; % 전체 배열 초기화
            else
                % -------------------------------------------------------
                % [3단계] Unsafe 시 새로운 sigma_pc 탐색
                % -------------------------------------------------------
                eqn = @(x) sqrt(r/obj.r_f_max) * cos((sigma_t + x)/2) - cos(x);
                
                try
                    sigma_pc = fzero(eqn, obj.sigma_ref_prev);
                    
                    y_calc_new = (sin(sigma_t_calc) - sin(sigma_pc)) .* (1 + cos(sigma_t_calc + sigma_pc)) ./ (cos(sigma_pc)^2 + dynamic_eps);
                    C_max = max(y_calc_new);

                    r_f_compare = (C_max * V_p^2) / (2 * obj.max_acc);
                    
                    if r_f_compare > obj.r_f_max

                        % -------------------------------------------------------
                        % [4단계] Fail-Safe 탈출
                        % -------------------------------------------------------

                        % gamma_t를 인자로 넘겨서 Target 방향에 맞게 궤적 생성
                        [x_traj, y_traj] = obj.get_Boundary(obj.sigma_pref, gamma_t);
                        
                        Distance_point = zeros(num_angles, 2); 
                        signed_distances = zeros(num_angles, 1);
                        abs_distances = zeros(num_angles, 1);
                        
                        for i = 1:num_angles
                            sigma_pc_val = deg2rad(obj.sigma_FRS_list(i)); 
                            
                            % 상대 운동 방정식 (이 부분은 원래 gamma_t에 대해 완벽히 일반화되어 있음)
                            dydt = @(t, Y) [
                                V_p * cos(atan2(-Y(2), -Y(1)) + sigma_pc_val) - V_t * cos(gamma_t);
                                V_p * sin(atan2(-Y(2), -Y(1)) + sigma_pc_val) - V_t * sin(gamma_t)
                            ];
                            
                            % obj.t_for 및 x_curr, y_curr 사용
                            [~, Y] = ode45(dydt, [0 obj.t_for], [x_curr; y_curr]); 
                            Distance_point(i, :) = Y(end, :); 
                            
                            % Trajectory까지의 최소 절대 거리 계산
                            if ~isempty(x_traj)
                                distances = sqrt((x_traj - Distance_point(i, 1)).^2 + (y_traj - Distance_point(i, 2)).^2);
                                abs_distances(i) = min(distances);
                            else
                                abs_distances(i) = inf;
                            end
                        end
                        
                        if ~isempty(x_traj)
                            % inpolygon을 사용하여 부호 결정 (영역 밖: +, 영역 안: -)
                            in = inpolygon(Distance_point(:,1), Distance_point(:,2), x_traj, y_traj);
                            for i = 1:num_angles
                                if in(i)
                                    signed_distances(i) = -abs_distances(i); % 안쪽이면 -
                                else
                                    signed_distances(i) = abs_distances(i);  % 바깥쪽이면 +
                                end
                            end
                            
                            % point_value 1열에 부호가 포함된 거리 저장
                            point_value(:, 1) = signed_distances;
                            point_value(:, 3:4) = Distance_point;

                            % 거리가 +에서 -로 바뀌는(Boundary를 통과하는) 첫 지점 탐색
                            % 1) 현재 스텝에서 Boundary 안으로 들어온 각도들에 true 부여
                            if signed_distances(1) <= 0
                                obj.is_latched(1) = true;
                            end
                            for i = 2:num_angles
                                if signed_distances(i-1) > 0 && signed_distances(i) <= 0
                                    obj.is_latched(i) = true;
                                end
                            end
                            
                            % 2) 지금까지 is_latched가 true로 누적된 인덱스들 찾기
                            latched_indices = find(obj.is_latched);
                            
                            if ~isempty(latched_indices)
                                % 누적된 후보군 배열 안에서 obj.sigma_pref 와의 차이가 제일 작은 인덱스 찾기
                                candidate_angles_rad = deg2rad(obj.sigma_FRS_list(latched_indices));
                                diffs = abs(candidate_angles_rad - obj.sigma_pref);
                                [~, min_diff_pos] = min(diffs);
                                
                                best_idx = latched_indices(min_diff_pos);
                                
                                sigma_pc = deg2rad(obj.sigma_FRS_list(best_idx));
                                mode_flag = 3; 
                                point_value(best_idx, 2) = 1; 
                            else
                                % 아직 아무것도 누적 안 된 경우 (후보가 없음)
                                [~, min_idx] = min(abs_distances);
                                sigma_pc = deg2rad(obj.sigma_FRS_list(min_idx));
                                mode_flag = 4;
                                point_value(min_idx, 2) = 1;
                            end
                        else
                            % Boundary 생성 실패 등 예외 상황
                            sigma_pc = obj.sigma_ref_prev;
                            mode_flag = 4;
                            % 거리를 알 수 없으므로 거리는 inf로 처리
                            point_value(:, 1) = inf;
                        end                      
                    else
                        mode_flag = 1;
                        obj.is_latched(:) = false; % Safe 판정이면 고정 해제
                    end
                catch
                    mode_flag = 2;
                    sigma_pc = obj.sigma_ref_prev;
                end
            end

            % -----------------------------------------------------------
            % [5단계] 유도 명령 계산 (Rate Limit 적용)
            % -----------------------------------------------------------
            
            % 최대 허용 변화량 계산
            max_delta = obj.rate_limit * obj.dt;
            delta_sigma = sigma_pc - obj.sigma_ref_prev;
            
            % Rate limit 제한 걸기
            if delta_sigma > max_delta
                sigma_pc_limited = obj.sigma_ref_prev + max_delta;
            elseif delta_sigma < -max_delta
                sigma_pc_limited = obj.sigma_ref_prev - max_delta;
            else
                sigma_pc_limited = sigma_pc;
            end
            
            sigma_ref_filtered = sigma_pc_limited;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end