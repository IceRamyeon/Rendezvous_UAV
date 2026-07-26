classdef RDPG_VT < handle
    properties
        % 공통/실제 타겟용 파라미터
        k, max_acc, r_f_max, dt
        sigma_ref_prev_real 
        
        % 가상 타겟(Virtual Target)용 파라미터
        r_allow_vt
        test_mode_vt
        sigma_ref_prev_vt
        
        % 가상 타겟 설정 위치
        r_vt 
        theta_vt_rad 
    end
    
    methods
        function obj = RDPG_VT(k_gain, limit_G, r_f_max, dt, init_sigma_real, ...
                               r_allow_vt, test_mode_vt, init_sigma_vt, ...
                               r_vt_input, theta_vt_rad_input)
                               
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
            obj.test_mode_vt = test_mode_vt;
            obj.sigma_ref_prev_vt = init_sigma_vt;
        end

        % 입력 인자에 실제 타겟 기구학과 가상 타겟 기구학을 모두 받도록
        function [acc_cmd, sigma_ref_new, mode_flag] = compute_command(obj, ...
            V_p, r, lambda_dot, sigma_p, sigma_t, ...         % Original Target Kinematics
            r_vt_dist, lambda_dot_vt, sigma_p_vt, sigma_t_vt) % Virtual Target Kinematics
            
            % =======================================================
            % Dynamic Epsilon 스케줄링 로직
            % =======================================================
            % epsilon이 매우 작을 경우, sigma_p0가 90도에 가까우면 수치적으로 불안정하여 sigma_pc 계산 실패할 가능성 있음
            % epsilon이 클 경우, 랑데부 궤적이 max_acc를 초과하는 정도가 커짐
            % 따라서 epsilon을 거리에 따라 조절
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
                % 종말 단계에서는 오차 없는 순정 상태 유지
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
                % 주어진 r_f로 랑데부에 성공하는 sigma_pc을 찾기
                eqn = @(x) sqrt(r/obj.r_f_max) * cos((sigma_t + x)/2) - cos(x);
                
                try
                    sigma_pc = fzero(eqn, obj.sigma_ref_prev_real);

                    % sigma_pc을 바탕으로 f(sigma_pc;sigma_t) = C 의 최댓값(C_max)을 계산
                    y_calc_new = (sin(sigma_t_calc) - sin(sigma_pc)) .* (1 + cos(sigma_t_calc + sigma_pc)) ./ (cos(sigma_pc)^2 + dynamic_eps);
                    C_max = max(y_calc_new);
                    % C = V_p^2 / (2 * a_max * r_f)에서, 주어진 a_max을 대입하여 r_f_compare 계산
                    r_f_compare = (C_max * V_p^2) / (2 * obj.max_acc);
                    
                    % r_f_compare가 주어진 r_f보다 작거나 같으면, sigma_pc로 만들어진 trajectory가 a_max을 넘지 않음.(Success-Safe)
                    % r_f_compare가 주어진 r_f보다 크면, sigma_pc로 만들어진 trajectory가 a_max을 초과하므로 탈출해야함.(Fail-Safe)
                    if r_f_compare > obj.r_f_max
                        mode_flag = 2;
                        sigma_pc = obj.sigma_ref_prev_real; 
                    else
                        mode_flag = 1;
                    end
                catch
                    mode_flag = 2;
                    sigma_pc = obj.sigma_ref_prev_real;
                end
            end

            % -----------------------------------------------------------
            % [4단계] Fail-Safe 탈출
            % -----------------------------------------------------------
            if mode_flag == 2
                % 가상 타겟(Virtual Target)에 대해 RDPG 사용
                eqn_vt = @(x) sqrt(r_vt_dist / obj.r_allow_vt) * cos((sigma_t_vt + x)/2) - cos(x);
                try
                    sigma_pc_vt = fzero(eqn_vt, obj.sigma_ref_prev_vt);
                catch
                    sigma_pc_vt = obj.sigma_ref_prev_vt;
                end
                
                if obj.test_mode_vt
                    sigma_pc_vt = min(sigma_pc_vt, sigma_t_vt);
                end
                
            % -----------------------------------------------------------
            % [5단계] 유도 명령 계산
            % -----------------------------------------------------------
                sigma_ref_new_vt = sigma_pc_vt;
                obj.sigma_ref_prev_vt = sigma_ref_new_vt;
                
                u_cmd = lambda_dot_vt - obj.k * (sigma_p_vt - sigma_ref_new_vt);
                sigma_ref_new = sigma_ref_new_vt;
                
            else
                % [Mode 0, 1] 실제 타겟(Real Target) RDPG_ACC 사용
                sigma_ref_filtered = sigma_pc;
                obj.sigma_ref_prev_real = sigma_ref_filtered;
                
                u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
                sigma_ref_new = sigma_ref_filtered;
            end
            
            % 가속도 제한
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end