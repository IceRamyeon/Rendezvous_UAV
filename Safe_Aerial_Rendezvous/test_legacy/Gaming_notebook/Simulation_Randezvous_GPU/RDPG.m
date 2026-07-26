classdef RDPG < handle
    % Reachability_DPG
    % 논문의 식 (15)를 통해 최적 리드각(sigma_ref)을 계산하고
    % 식 (16)으로 변화율을 제한하여 DPG 유도 명령을 생성함.
    
    properties
        k           % DPG 게인
        max_acc     % 가속도 제한
        
        % Reachability 파라미터
        r_allow     % 목표 허용 거리 (m)
        rate_limit  % 각도 변화율 제한 (rad/s)
        dt          % 시뮬레이션 Time Step
        
        % 상태 저장용
        sigma_ref_prev % 이전 스텝의 참조 리드각
    end
    
    methods
        %% 생성자
        function obj = RDPG(k_gain, limit_G, r_allow, rate_limit, dt, init_sigma_rad)
            if nargin < 1, k_gain = 3.0; end
            if nargin < 6, init_sigma_rad = 0; end % 초기값 없으면 0
            
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            
            obj.r_allow = r_allow;
            obj.rate_limit = rate_limit;
            obj.dt = dt;
            
            obj.sigma_ref_prev = init_sigma_rad; % 초기화
        end

        %% 메인 유도 명령 계산 함수
        % 입력: V_p, lambda_dot, sigma_p, r(거리), sigma_t(타겟리드각)
        function [acc_cmd, sigma_ref_curr] = compute_commandRDPG(obj, V_p, lambda_dot, sigma_p, r, sigma_t)
            
            % --- [Step 1] 식 (15): Reference Calculation (해 찾기) ---
            % 방정식: r * cos^2((sigma_t + x)/2) - r_allow * cos^2(x) = 0
            % x에 대해 풉니다.
            eqn = @(x) r * cos((sigma_t + x)/2)^2 - obj.r_allow * cos(x)^2;
            
            try
                % 이전 값을 초기 추정치로 사용하여 해를 찾음
                x_candidate = fzero(eqn, obj.sigma_ref_prev);
            catch
                % 해를 못 찾으면 이전 값 유지 (Fail-safe)
                x_candidate = obj.sigma_ref_prev;
            end
            
            % --- [Step 2] 식 (16): Rate Limiter (변화율 제한) ---
            delta = x_candidate - obj.sigma_ref_prev;
            max_change = obj.rate_limit * obj.dt;
            
            if abs(delta) > max_change
                delta = sign(delta) * max_change;
            end
            
            sigma_ref_curr = obj.sigma_ref_prev + delta;
            
            % 상태 업데이트 (다음 스텝을 위해 저장)
            obj.sigma_ref_prev = sigma_ref_curr;
            
            % --- [Step 3] DPG Command Generation ---
            % u = lambda_dot - K * (sigma_p - sigma_ref)
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_curr);
            
            % 가속도 변환 및 제한
            raw_acc = V_p * u_cmd;
            
            if abs(raw_acc) > obj.max_acc
                acc_cmd = obj.max_acc * sign(raw_acc);
            else
                acc_cmd = raw_acc;
            end
        end
    end
end