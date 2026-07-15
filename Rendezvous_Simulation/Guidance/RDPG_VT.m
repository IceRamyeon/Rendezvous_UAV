classdef RDPG_VT < handle
    properties
        % 공통/실제 타겟용 파라미터
        k, max_acc, r_f_max, dt
        sigma_ref_prev_real 
        
        % 가상 타겟(Virtual Target)용 파라미터
        r_allow_vt, rate_limit_vt
        alpha_vt    % LPF 계수
        test_mode_vt
        sigma_ref_prev_vt
        
        % 가상 타겟 설정 위치 (생성자에서 입력받음)
        r_vt 
        theta_vt_rad 
    end
    
    methods
        function obj = RDPG_VT(k_gain, limit_G, r_f_max, dt, init_sigma_real, ...
                               r_allow_vt, rate_limit_vt, alpha_vt, test_mode_vt, init_sigma_vt, ...
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
            
            if nargin < 8, obj.alpha_vt = 1.0; else, obj.alpha_vt = alpha_vt; end
        end

        function [acc_cmd, sigma_ref_new, mode_flag, x_vt, y_vt] = compute_command(obj, x_p, y_p, V_p, chi_p, x_t, y_t, V_t, chi_t)
            
            % -----------------------------------------------------------
            % [0단계] 실제 타겟(Real Target)에 대한 상대 운동학 계산
            % -----------------------------------------------------------
            dx_real = x_t - x_p;
            dy_real = y_t - y_p;
            r_real = norm([dx_real, dy_real]);
            
            lambda_real = atan2(dy_real, dx_real);
            
            % 상대 속도 (헤딩이 +y축 기준 CW 방향인 경우)
            V_rel_x_real = V_t * sin(chi_t) - V_p * sin(chi_p);
            V_rel_y_real = V_t * cos(chi_t) - V_p * cos(chi_p);
            
            lambda_dot_real = (dx_real * V_rel_y_real - dy_real * V_rel_x_real) / (r_real^2);
            sigma_p_real = chi_p - lambda_real;
            sigma_t_real = chi_t - lambda_real;
            
            % -----------------------------------------------------------
            % [1단계] 동적 Epsilon 스케줄링 및 실제 타겟 Reachable Region 판별
            % -----------------------------------------------------------
            eps_max = 0.01;
            eps_min = 1e-7;
            r_fade_start = 200.0; 
            r_fade_end = 100.0;    
            
            if r_real >= r_fade_start
                dynamic_eps = eps_max;
            elseif r_real > r_fade_end
                alpha_eps = (r_real - r_fade_end) / (r_fade_start - r_fade_end);
                dynamic_eps = eps_min + alpha_eps * (eps_max - eps_min);
            else
                dynamic_eps = eps_min;
            end
            
            sigma_t_calc = linspace(0, pi, 1000); 
            y_calc = (sin(sigma_t_calc) - sin(sigma_p_real)) .* (1 + cos(sigma_t_calc + sigma_p_real)) ./ (cos(sigma_p_real)^2 + dynamic_eps);
            y_max = max(y_calc);
            
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            denom_contour = cos((sigma_t_real + sigma_p_real)/2)^2 + dynamic_eps;
            r_contour_min = r_f_min * (cos(sigma_p_real))^2 / denom_contour;
            r_contour_max = obj.r_f_max * (cos(sigma_p_real))^2 / denom_contour;

            % Safe / Unsafe 판별 로직
            if r_real >= r_contour_min && r_real <= r_contour_max
                x_candidate_real = sigma_p_real;
                mode_flag = 0; 
            else
                eqn_real = @(x) sqrt(r_real/obj.r_f_max) * cos((sigma_t_real + x)/2) - cos(x);
                try
                    x_candidate_real = fzero(eqn_real, obj.sigma_ref_prev_real);
                    
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate_real)) .* (1 + cos(sigma_t_calc + x_candidate_real)) ./ (cos(x_candidate_real)^2 + dynamic_eps);
                    y_max_new = max(y_calc_new);
                    r_f_min_new = (y_max_new * V_p^2) / (2 * obj.max_acc);
                    
                    if r_f_min_new > obj.r_f_max
                        mode_flag = 2;
                        x_candidate_real = obj.sigma_ref_prev_real; 
                    else
                        mode_flag = 1;
                    end
                catch
                    mode_flag = 2;
                    x_candidate_real = obj.sigma_ref_prev_real;
                end
            end

            % -----------------------------------------------------------
            % [2단계] Mode Flag에 따른 분기 (Virtual Target vs Real Target)
            % -----------------------------------------------------------
            if mode_flag == 2
                % [Virtual Target 유도 모드] - RDPG 로직 사용
                
                % 가상 타겟의 전역 좌표 (Target Heading 기준)
                x_vt = x_t + obj.r_vt * sin(chi_t + obj.theta_vt_rad);
                y_vt = y_t + obj.r_vt * cos(chi_t + obj.theta_vt_rad);
                
                % 가상 타겟의 속도는 실제 타겟과 동일하다고 가정
                vx_vt = V_t * sin(chi_t);
                vy_vt = V_t * cos(chi_t);
                
                dx_vt = x_vt - x_p;
                dy_vt = y_vt - y_p;
                r_vt_rel = norm([dx_vt, dy_vt]);
                
                lambda_vt = atan2(dy_vt, dx_vt);
                V_rel_x_vt = vx_vt - (V_p * sin(chi_p));
                V_rel_y_vt = vy_vt - (V_p * cos(chi_p));
                
                lambda_dot_vt = (dx_vt * V_rel_y_vt - dy_vt * V_rel_x_vt) / (r_vt_rel^2);
                
                sigma_p_vt = chi_p - lambda_vt;
                sigma_t_vt = chi_t - lambda_vt; 
                
                % 최적 리드각 계산
                eqn_vt = @(x) sqrt(r_vt_rel/obj.r_allow_vt) * cos((sigma_t_vt + x)/2) - cos(x);
                try
                    x_candidate_vt = fzero(eqn_vt, obj.sigma_ref_prev_vt);
                catch
                    x_candidate_vt = obj.sigma_ref_prev_vt;
                end
                
                if obj.test_mode_vt
                    x_candidate_vt = min(x_candidate_vt, sigma_t_vt);
                end

                % 변화율 제한(Rate Limit) 적용
                delta_vt = x_candidate_vt - obj.sigma_ref_prev_vt;
                max_change_vt = obj.rate_limit_vt * obj.dt;
                if abs(delta_vt) > max_change_vt
                    delta_vt = sign(delta_vt) * max_change_vt;
                end
                sigma_ref_new_vt = obj.sigma_ref_prev_vt + delta_vt;

                % LPF 적용
                sigma_ref_filtered_vt = obj.alpha_vt * sigma_ref_new_vt + (1 - obj.alpha_vt) * obj.sigma_ref_prev_vt;
                obj.sigma_ref_prev_vt = sigma_ref_filtered_vt;

                sigma_ref_new = sigma_ref_filtered_vt;
                
                % 가속도 명령 생성
                u_cmd = lambda_dot_vt - obj.k * (sigma_p_vt - sigma_ref_filtered_vt);
                raw_acc = V_p * u_cmd;
                acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
                
            else
                % [Real Target 유도 모드] - RDPG_ACC 로직 사용
                
                x_vt = NaN; y_vt = NaN; % 가상 타겟 미사용
                
                sigma_ref_filtered_real = x_candidate_real;
                obj.sigma_ref_prev_real = sigma_ref_filtered_real;

                sigma_ref_new = sigma_ref_filtered_real;
                
                u_cmd = lambda_dot_real - obj.k * (sigma_p_real - sigma_ref_filtered_real);
                raw_acc = V_p * u_cmd;
                acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
            end
        end
    end
end