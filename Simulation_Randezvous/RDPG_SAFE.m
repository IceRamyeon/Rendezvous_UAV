classdef RDPG_SAFE < handle
    % Reachability_DPG_SAFE
    % 논문의 식 (15)를 통해 최적 리드각(sigma_ref)을 계산하되, (CBF B안 적용)
    % 가속도 한계를 넘지 않도록 Reference Governor로 필터링함.
    % 이후 CLF-QP를 사용하여 최종 가속도(acc_cmd)를 도출함.
    
    properties
        k           % DPG 게인 (QP의 u_ref 생성용)
        max_acc     % 가속도 제한 (m/s^2)
        V_p         % Pursuer 속도
        V_t         % Target 속도
        
        % Reachability 파라미터
        r_allow     % 목표 허용 거리 (m)
        rate_limit  % 각도 변화율 제한 (rad/s)
        dt          % 시뮬레이션 Time Step
        
        % 상태 저장용
        sigma_ref_prev % 이전 스텝의 참조 리드각
        
        % QP 튜닝 하이퍼파라미터
        lambda_clf  % CLF 수렴 속도
        p_weight    % Slack 변수 페널티 가중치
        
        % 으헤~ 무거운 로깅 배열들은 싹 치워버렸어!
    end
    
    methods
        %% 생성자
        function obj = RDPG_SAFE(k_gain, limit_G, r_allow, rate_limit, dt, init_sigma_rad, V_p, V_t)
            if nargin < 1, k_gain = 3.0; end
            if nargin < 6, init_sigma_rad = 0; end % 초기값 없으면 0
            
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81; % G to m/s^2
            
            obj.r_allow = r_allow;
            obj.rate_limit = rate_limit;
            obj.dt = dt;
            
            obj.sigma_ref_prev = init_sigma_rad; % 초기화
            obj.V_p = V_p;
            obj.V_t = V_t;
            
            % QP 튜닝 변수
            obj.lambda_clf = 5.0; 
            obj.p_weight = 1000.0;
        end
        
        %% 메인 유도 명령 계산 함수 (CBF-CLF-QP)
        % [수정됨] 출력에 V_clf, H_cbf, slack_opt를 추가로 반환!
        function [acc_cmd, sigma_ref_curr, V_clf, H_cbf, slack_opt] = compute_commandRDPG_SAFE(obj, X_State, lambda_dot)
            % 1. 상태 변수 분해
            r = X_State(1);
            sigma_t = X_State(2);
            sigma_p = X_State(3);
            
            % =========================================================
            % [Step 1] Reference Governor: 안전한 sigma_ref 계산 (CBF B안)
            % =========================================================
            eqn = @(x) r * cos((sigma_t + x)/2)^2 - obj.r_allow * cos(x)^2;
            try
                x_candidate = fzero(eqn, obj.sigma_ref_prev);
            catch
                x_candidate = obj.sigma_ref_prev;
            end
            
            safe_margin = (r * obj.max_acc) / (obj.V_p^2);
            sin_ref_min = sin(sigma_t) - safe_margin;
            sin_ref_max = sin(sigma_t) + safe_margin;
            
            sin_cand = sin(x_candidate);
            
            if sin_cand > sin_ref_max
                target_sin = max(min(sin_ref_max, 1), -1);
                if cos(x_candidate) >= 0
                    x_candidate = asin(target_sin);
                else
                    x_candidate = pi - asin(target_sin); 
                end
            elseif sin_cand < sin_ref_min
                target_sin = max(min(sin_ref_min, 1), -1);
                if cos(x_candidate) >= 0
                    x_candidate = asin(target_sin);
                else
                    x_candidate = -pi - asin(target_sin); 
                end
            end
            
            x_candidate = wrapToPi(x_candidate);
            
            % =========================================================
            % [Step 2] Rate Limiter (변화율 제한, 식 16)
            % =========================================================
            delta = x_candidate - obj.sigma_ref_prev;
            max_change = obj.rate_limit * obj.dt;
            if abs(delta) > max_change
                delta = sign(delta) * max_change;
            end
            sigma_ref_curr = obj.sigma_ref_prev + delta;
            obj.sigma_ref_prev = sigma_ref_curr; 
            
            % =========================================================
            % [Step 3] CLF-QP Formulation
            % =========================================================
            u_ref = obj.V_p * (lambda_dot - obj.k * (sigma_p - sigma_ref_curr));
            
            V_clf = (sigma_p - sigma_ref_curr)^2;
            LfV = -2 * (sigma_p - sigma_ref_curr) * lambda_dot; 
            LgV = 2 * (sigma_p - sigma_ref_curr) / obj.V_p;
            
            H_qp = [1, 0; 
                    0, 2 * obj.p_weight];
            f_qp = [-u_ref; 0];
            
            A_ineq = [LgV, -1];
            B_ineq = -LfV - obj.lambda_clf * V_clf;
            
            LB = [-obj.max_acc; 0];
            UB = [obj.max_acc; Inf];
            
            options = optimset('Display', 'off');
            [X_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, B_ineq, [], [], LB, UB, [], options);
            
            if exitflag == 1 || exitflag == 2
                acc_cmd = X_opt(1);
                slack_opt = X_opt(2);
            else
                acc_cmd = max(min(u_ref, obj.max_acc), -obj.max_acc);
                slack_opt = 0;
            end
            
            % =========================================================
            % [Step 4] 출력할 CBF 안전 마진 계산 (H_cbf)
            % =========================================================
            a_geom = (obj.V_p^2 / r) * (sin(sigma_t) - sin(sigma_ref_curr));
            H_cbf = obj.max_acc - abs(a_geom); 
            
            % 값들은 함수의 출력(Return)을 통해 메인 코드로 넘어가게 돼~
        end
    end
end