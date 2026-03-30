classdef RDPG < handle
    properties
        k, max_acc, r_allow, rate_limit, dt
        sigma_ref_prev 
        alpha % [추가] LPF 계수
        test_mode
    end
    
    methods
        function obj = RDPG(k_gain, limit_G, r_allow, rate_limit, dt, init_sigma_rad, alpha, test_mode)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_allow = r_allow;
            obj.rate_limit = rate_limit;
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
            obj.test_mode = test_mode;
            % alpha 값이 없으면 기본값 1.0 (필터링 없음)
            if nargin < 7, obj.alpha = 1.0; else, obj.alpha = alpha; end
        end

        function [acc_cmd, sigma_ref_filtered] = compute_commandRDPG(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            % [Step 1] 최적 리드각 계산
            eqn = @(x) sqrt(r/obj.r_allow) * cos((sigma_t + x)/2) - cos(x);
            try
                x_candidate = fzero(eqn, obj.sigma_ref_prev);
            catch
                x_candidate = obj.sigma_ref_prev;
            end
            
            if obj.test_mode
            x_candidate = min(x_candidate, sigma_t);
            end

            % [Step 2] 변화율 제한
            delta = x_candidate - obj.sigma_ref_prev;
            max_change = obj.rate_limit * obj.dt;
            if abs(delta) > max_change
                delta = sign(delta) * max_change;
            end
            sigma_ref_new = obj.sigma_ref_prev + delta;

            % [Step 2.5] LPF 적용
            sigma_ref_filtered = obj.alpha * sigma_ref_new + (1 - obj.alpha) * obj.sigma_ref_prev;
            obj.sigma_ref_prev = sigma_ref_filtered;
            
            % [Step 3] DPG 명령 생성
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end