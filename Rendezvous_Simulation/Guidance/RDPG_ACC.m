classdef RDPG_ACC < handle
    properties
        k, max_acc, r_f_max, rate_limit, dt
        sigma_ref_prev 
        alpha 
        test_mode
    end
    
    methods
        function obj = RDPG_MAX(k_gain, limit_G, r_f_max, rate_limit, dt, init_sigma_rad, alpha, test_mode)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_f_max = r_f_max; % 기존 r_allow를 r_f_max(2m)로 활용
            obj.rate_limit = rate_limit;
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
            obj.test_mode = test_mode;
            if nargin < 7, obj.alpha = 1.0; else, obj.alpha = alpha; end
        end

        function [acc_cmd, sigma_ref_filtered, mode_flag] = compute_command(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            % -----------------------------------------------------------
            % [1단계 & 2단계] 현재 Pursuer 리드각(sigma_p) 기준 경계선 도출
            % -----------------------------------------------------------
            % 실시간 연산 속도를 위해 샘플 수는 1000개로 조정
            sigma_t_calc = linspace(0, pi, 1000); 
            y_calc = (sin(sigma_t_calc) - sin(sigma_p)) .* (1 + cos(sigma_t_calc + sigma_p)) ./ (cos(sigma_p)^2 + 1e-7);
            y_max = max(y_calc);
            
            % 현재 리드각에서의 최소 요구 반경(r_f_min) 계산
            r_f_min = (y_max * V_p^2) / (2 * obj.max_acc);

            % 현재 위치에 대응하는 f1, f2 경계 반경 계산
            denom = cos((sigma_t + sigma_p)/2)^2 + 1e-7;
            r_contour_min = r_f_min * (cos(sigma_p))^2 / denom;
            r_contour_max = obj.r_f_max * (cos(sigma_p))^2 / denom;

            % -----------------------------------------------------------
            % [3단계] 현재 위치가 f1 * f2 <= 0 (경계 사이)에 있는지 검토
            % -----------------------------------------------------------
            if r >= r_contour_min && r <= r_contour_max
                % [Safe 영역] 현재 리드각을 유지 (u_cmd가 lambda_dot에 가까워짐)
                x_candidate = sigma_p;
                mode_flag = 0; 
            else
                % -------------------------------------------------------
                % [4단계] Unsafe 영역 진입 -> r_f = 2m 만족하는 새로운 sigma_pc 탐색
                % -------------------------------------------------------
                eqn = @(x) sqrt(r/obj.r_f_max) * cos((sigma_t + x)/2) - cos(x);
                try
                    x_candidate = fzero(eqn, obj.sigma_ref_prev);
                    
                    % ---------------------------------------------------
                    % [5단계] 예외 처리 -> 구한 해가 No Intersect 영역인지 검토
                    % ---------------------------------------------------
                    y_calc_new = (sin(sigma_t_calc) - sin(x_candidate)) .* (1 + cos(sigma_t_calc + x_candidate)) ./ (cos(x_candidate)^2 + 1e-7);
                    y_max_new = max(y_calc_new);
                    r_f_min_new = (y_max_new * V_p^2) / (2 * obj.max_acc);
                    
                    if r_f_min_new > obj.r_f_max
                        % 해가 도달 불가능 영역에 있음 -> Fallback DPG 모드 전환
                        mode_flag = 2;
                        x_candidate = obj.sigma_ref_prev; 
                    else
                        % 성공적으로 새로운 해 선택 완료
                        mode_flag = 1;
                    end
                catch
                    % fzero 탐색 실패 시 예외 처리 -> Fallback DPG 모드 전환
                    mode_flag = 2;
                    x_candidate = obj.sigma_ref_prev;
                end
            end

            if obj.test_mode
                x_candidate = min(x_candidate, sigma_t);
            end

            % -----------------------------------------------------------
            % [6단계] 새로운 sigma_pc 적용 및 변화율 제한 / 필터링
            % -----------------------------------------------------------
            delta = x_candidate - obj.sigma_ref_prev;
            max_change = obj.rate_limit * obj.dt;
            if abs(delta) > max_change
                delta = sign(delta) * max_change;
            end
            sigma_ref_new = obj.sigma_ref_prev + delta;

            % LPF 적용
            sigma_ref_filtered = obj.alpha * sigma_ref_new + (1 - obj.alpha) * obj.sigma_ref_prev;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            % 최종 유도 명령 생성 및 가속도 제한 적용
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end