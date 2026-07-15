classdef RDPG_VT < handle
    properties
        % 공통/실제 타겟용 파라미터
        k, max_acc, r_f_max, dt
        sigma_ref_prev_real 
        
        % 가상 타겟(Virtual Target)용 파라미터
        r_allow_vt, rate_limit_vt
        test_mode_vt
        sigma_ref_prev_vt
        
        % 가상 타겟 설정 위치 (생성자에서 입력받음)
        r_vt 
        theta_vt_rad 
    end
    
    methods
        function obj = RDPG_VT(k_gain, limit_G, r_f_max, dt, init_sigma_real, ...
                               r_allow_vt, rate_limit_vt, test_mode_vt, init_sigma_vt, ...
                               r_vt_input, theta_vt_rad_input) % <--- 가상 타겟 위치 인자 추가
                               
            % 공통 및 실제 타겟 초기화
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; 
            obj.dt = dt;
            obj.sigma_ref_prev_real = init_sigma_real;
            
            % 가상 타겟 설정 위치 저장
            obj.r_vt = r_vt_input;
            obj.theta_vt_rad = theta_vt_rad_input;
            
            % 가상 타겟 초기화
            obj.r_allow_vt = r_allow_vt;
            obj.rate_limit_vt = rate_limit_vt;
            obj.test_mode_vt = test_mode_vt;
            obj.sigma_ref_prev_vt = init_sigma_vt;
            
        end

        function [acc_cmd, sigma_ref_new, mode_flag] = compute_command(obj, x_p, y_p, V_p, chi_p, x_t, y_t, V_t, chi_t, lambda_dot_vt)
            
            % epsilon 스케줄링 로직
            eps_max = 0.01;
            eps_min = 1e-7;
            r_fade_start = 200.0; % epsilon 최대 거리 (시뮬레이션 보며 조절)
            r_fade_end = 100.0;    % epsilon 최소 거리 (시뮬레이션 보며 조절)
            if r >= r_fade_start
                dynamic_eps = eps_max;
            elseif r > r_fade_end
                % r_start ~ r_end 사이에서는 선형적으로 부드럽게 감소
                alpha = (r - r_fade_end) / (r_fade_start - r_fade_end);
                dynamic_eps = eps_min + alpha * (eps_max - eps_min);
            else
                % 종말 단계에서는 epsilon min
                dynamic_eps = eps_min;
            end

            % -----------------------------------------------------------
            % [1단계] r_f_min 계산하여 현재 Pursuer가 Reachable Region 내에 있는지 확인
            % -----------------------------------------------------------
            sigma_t_calc = linspace(0, pi, 1000); 
            
            % r_f_min 계산
            y_calc = (sin(sigma_t_calc) - sin(sigma_p)) .* (1 + cos(sigma_t_calc + sigma_p)) ./ (cos(sigma_p)^2 + dynamic_eps);
            y_max = max(y_calc);
            
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            % 여기 contour 분모에도 1e-7 대신 dynamic_eps로 통일
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
                    
                    % 
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate)) .* (1 + cos(sigma_t_calc + x_candidate)) ./ (cos(x_candidate)^2 + dynamic_eps);
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