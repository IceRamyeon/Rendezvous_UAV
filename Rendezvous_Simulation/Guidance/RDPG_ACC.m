classdef RDPG_ACC < handle
    properties
        k, max_acc, r_f_max, dt
        sigma_ref_prev 
    end
    
    methods
        function obj = RDPG_ACC(k_gain, limit_G, r_f_max, dt, init_sigma_rad)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; % 기존 r_allow를 r_f_max(2m)로 활용
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            % -----------------------------------------------------------
            % [1단계 & 2단계] 경계선 도출
            % -----------------------------------------------------------
            sigma_t_calc = linspace(0, pi, 1000); 
            y_calc = (sin(sigma_t_calc) - sin(sigma_p)) .* (1 + cos(sigma_t_calc + sigma_p)) ./ (cos(sigma_p)^2 + 1e-7);
            y_max = max(y_calc);
            
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            denom = cos((sigma_t + sigma_p)/2)^2 + 1e-7;
            r_contour_min = r_f_min * (cos(sigma_p))^2 / denom;
            r_contour_max = obj.r_f_max * (cos(sigma_p))^2 / denom;

            % -----------------------------------------------------------
            % [3단계] Safe / Unsafe 판별
            % -----------------------------------------------------------
            if r >= r_contour_min && r <= r_contour_max
                x_candidate = sigma_p;
                mode_flag = 0; 
            else
                % -------------------------------------------------------
                % [4단계 & 5단계] Unsafe 시 새로운 sigma_pc 탐색
                % -------------------------------------------------------
                eqn = @(x) sqrt(r/obj.r_f_max) * cos((sigma_t + x)/2) - cos(x);
                try
                    x_candidate = fzero(eqn, obj.sigma_ref_prev);
                    
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate)) .* (1 + cos(sigma_t_calc + x_candidate)) ./ (cos(x_candidate)^2 + 1e-7);
                    y_max_new = max(y_calc_new);
                    r_f_min_new = (y_max_new * V_p^2) / (2 * obj.max_acc);
                    
                    if r_f_min_new > obj.r_f_max
                        mode_flag = 2;
                        x_candidate = obj.sigma_ref_prev; 
                    else
                        mode_flag = 1;
                    end
                catch
                    mode_flag = 2;
                    x_candidate = obj.sigma_ref_prev;
                end
            end

            % -----------------------------------------------------------
            % [6단계] 필터링/변화율 제한 없이 결과값 할당
            % -----------------------------------------------------------
            % 필터링 없이 x_candidate를 바로 결과값으로 사용
            sigma_ref_filtered = x_candidate;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            % 유도 명령 계산
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end