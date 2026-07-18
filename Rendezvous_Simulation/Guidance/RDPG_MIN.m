classdef RDPG_MIN < handle
    properties
        k, max_acc, r_f_max, dt
        sigma_ref_prev 
        min_acc % [수정] 클래스 속성에 min_acc 추가
    end
    
    methods
        function obj = RDPG_MIN(k_gain, limit_G, min_G, r_f_max, dt, init_sigma_rad)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.min_acc = min_G * 9.81; % [주의] 루트 계산을 위해 양수 유지
            obj.r_f_max = r_f_max; 
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            
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
                x_candidate = sigma_p;
                mode_flag = 0; 
            else
                % -------------------------------------------------------
                % [3단계] Unsafe 시 새로운 sigma_pc 탐색
                % -------------------------------------------------------
                eqn = @(x) sqrt(r/obj.r_f_max) * cos((sigma_t + x)/2) - cos(x);
                
                try
                    x_candidate = fzero(eqn, obj.sigma_ref_prev);
                    
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate)) .* (1 + cos(sigma_t_calc + x_candidate)) ./ (cos(x_candidate)^2 + dynamic_eps);
                    y_max_new = max(y_calc_new);
                    r_f_min_new = (y_max_new * V_p^2) / (2 * obj.max_acc);
                    
                    if r_f_min_new > obj.r_f_max
                        mode_flag = 2;
                        % mode_flag = 2일 때, min_acc가 그리는 영역과 접하는 r_f로 랑데부 시도
                        r_f_min_escape = (y_max_new * V_p^2) / (2 * obj.min_acc);

                        eqn_escape = @(x) sqrt(r/r_f_min_escape) * cos((sigma_t + x)/2) - cos(x);
                        try 
                            x_candidate = fzero(eqn_escape, obj.sigma_ref_prev);
                        catch
                            x_candidate = obj.sigma_ref_prev; 
                        end
                        
                        % [수정] 이 위치에 있던 x_candidate = obj.sigma_ref_prev; 삭제 완료
                    else
                        mode_flag = 1;
                    end
                catch
                    mode_flag = 2;
                    x_candidate = obj.sigma_ref_prev;                    
                end
            end

            % -----------------------------------------------------------
            % [4단계] 유도 명령 계산
            % -----------------------------------------------------------
            sigma_ref_filtered = x_candidate;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end