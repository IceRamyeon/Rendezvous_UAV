classdef RDPG_ACC < handle
    properties
        k, max_acc, r_f_max, dt
        sigma_ref_prev 
    end
    
    methods
        function obj = RDPG_ACC(k_gain, limit_G, r_f_max, dt, init_sigma_rad)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; 
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            
            % =======================================================
            % [추가] 동적 Epsilon 스케줄링 로직
            % =======================================================
            eps_max = 0.01;
            eps_min = 1e-7;
            r_fade_start = 300.0; % 거품 빼기 시작할 거리 (시뮬레이션 보며 조절)
            r_fade_end = 100.0;    % 거품 완전히 제거할 거리 (시뮬레이션 보며 조절)
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
            % [1단계 & 2단계] 경계선 도출
            % -----------------------------------------------------------
            sigma_t_calc = linspace(0, pi, 1000); 
            
            % 기존 1e-7 대신 dynamic_eps 적용
            y_calc = (sin(sigma_t_calc) - sin(sigma_p)) .* (1 + cos(sigma_t_calc + sigma_p)) ./ (cos(sigma_p)^2 + dynamic_eps);
            y_max = max(y_calc);
            
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            % 여기 contour 분모에도 1e-7 대신 dynamic_eps로 통일
            denom_contour = cos((sigma_t + sigma_p)/2)^2 + dynamic_eps;
            r_contour_min = r_f_min * (cos(sigma_p))^2 / denom_contour;
            r_contour_max = obj.r_f_max * (cos(sigma_p))^2 / denom_contour;

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
                % (앞서 논의한 목표 조준점 빡빡하게 잡는 로직 적용 예시)
                tight_r = obj.r_f_max * 1.0; % 필요에 따라 0.7~0.9 사이 조절
                eqn = @(x) sqrt(r/tight_r) * cos((sigma_t + x)/2) - cos(x);
                
                try
                    x_candidate = fzero(eqn, obj.sigma_ref_prev);
                    
                    % 새로운 x_candidate 검증 시에도 dynamic_eps 적용!
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate)) .* (1 + cos(sigma_t_calc + x_candidate)) ./ (cos(x_candidate)^2 + dynamic_eps);
                    y_max_new = max(y_calc_new);
                    r_f_min_new = (y_max_new * V_p^2) / (2 * obj.max_acc);
                    
                    % 포기 판정은 타이트한 반경이 아니라 원래 반경(r_f_max) 기준
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
            % [6단계] 유도 명령 계산
            % -----------------------------------------------------------
            sigma_ref_filtered = x_candidate;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end