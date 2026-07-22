classdef RDPG_FRS < handle
    properties
        k, max_acc, r_f_max, dt
        sigma_ref_prev
        sigma_pref % 62.6도와 같은 선호 각도
        t_for      % Fail-Safe 탐색을 위한 전방향 시뮬레이션 시간
    end
    
    methods
        function obj = RDPG_FRS(k_gain, limit_G, r_f_max, dt, init_sigma_rad, sigma_pref, t_for)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; 
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
            obj.sigma_pref = sigma_pref;
            obj.t_for = t_for; % 생성자에서 t_for 입력받음
        end

        function [x_traj, y_traj] = get_Trajectory(obj, sigma_pref)
            % 주어진 sigma_pref로 One_sigma_pc.m의 Boundary line의 점들 생성 및 저장   
            r_f = obj.r_f_max;
            max_r_plot_limit = 20000;  

            sigma_pc_deg_traj = sigma_pref;
            sigma_pc_rad_traj = deg2rad(sigma_pc_deg_traj);

            % 해석적 궤적 계산
            sigma_t_traj = linspace(-sigma_pc_rad_traj, pi - sigma_pc_rad_traj - 1e-5, 2000);
            denominator = cos((sigma_t_traj + sigma_pc_rad_traj) / 2).^2;
            r_traj = r_f * cos(sigma_pc_rad_traj)^2 ./ denominator;
            r_traj = min(r_traj, max_r_plot_limit);

            valid_idx = isfinite(r_traj) & (r_traj >= r_f);
            plot_sigma_t = sigma_t_traj(valid_idx);
            plot_r = r_traj(valid_idx);

            x_traj = [];
            y_traj = [];
            
            if ~isempty(plot_r)
                plot_angle = -pi/2 - plot_sigma_t;
                x_traj = plot_r .* cos(plot_angle);
                y_traj = plot_r .* sin(plot_angle);
            end
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t, V_t, gamma_t, x_curr, y_curr)
            
            sigma_pc_deg_set = -170:10:180; 

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
                        
                        [x_traj, y_traj] = obj.get_Trajectory(obj.sigma_pref);
                        
                        final_points = zeros(length(sigma_pc_deg_set), 2); 
                        
                        for i = 1:length(sigma_pc_deg_set)
                            sigma_pc_val = deg2rad(sigma_pc_deg_set(i)); 
                            
                            % 상대 운동 방정식
                            dydt = @(t, Y) [
                                V_p * cos(atan2(-Y(2), -Y(1)) + sigma_pc_val) - V_t * cos(gamma_t);
                                V_p * sin(atan2(-Y(2), -Y(1)) + sigma_pc_val) - V_t * sin(gamma_t)
                            ];
                            
                            % obj.t_for 및 x_curr, y_curr 사용
                            [~, Y] = ode45(dydt, [0 obj.t_for], [x_curr; y_curr]); 
                            final_points(i, :) = Y(end, :); 
                        end
                        
                        in = inpolygon(final_points(:,1), final_points(:,2), x_traj, y_traj);

                        if any(in) && any(~in)
                            % Region 경계를 지나는 경우: Region 내부 점들 중 sigma_pref에 가장 가까운 것 선택
                            valid_idx = find(in);
                            [~, min_idx] = min(abs(sigma_pc_deg_set(valid_idx) - obj.sigma_pref));
                            best_idx = valid_idx(min_idx);
                            
                            sigma_pc = deg2rad(sigma_pc_deg_set(best_idx));
                            mode_flag = 3; 
                        else
                            % 점들이 완전히 Region 밖(또는 안)에 있는 경우: get_Trajectory 궤적과 거리가 가장 짧은 점의 sigma_pc 선택
                            min_dist = inf; 
                            best_fp_idx = 1; 
                            
                            if ~isempty(x_traj)
                                for i = 1:size(final_points, 1) 
                                    % Final Point와 궤적 상의 모든 점 사이의 거리 계산
                                    distances = sqrt((x_traj - final_points(i, 1)).^2 + (y_traj - final_points(i, 2)).^2); 
                                    
                                    local_min_dist = min(distances); 
                                    
                                    if local_min_dist < min_dist 
                                        min_dist = local_min_dist; 
                                        best_fp_idx = i; 
                                    end
                                end
                            end
                            
                            sigma_pc = deg2rad(sigma_pc_deg_set(best_fp_idx));
                            mode_flag = 4;
                        end                        
                    else
                        mode_flag = 1;
                    end
                catch
                    mode_flag = 2;
                    sigma_pc = obj.sigma_ref_prev;
                end
            end

            % -----------------------------------------------------------
            % [5단계] 유도 명령 계산
            % -----------------------------------------------------------
            sigma_ref_filtered = sigma_pc;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end