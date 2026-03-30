classdef RDPG_SAFE < handle
    % Reachability_DPG_SAFE
    % 논문의 식을 통해 최적 리드각(sigma_ref)을 계산하되,
    % 궤적이 Max Acc Region과 겹치면(위험지역) CBF 조건을 통해
    % 가속도를 안전한 방향으로 보정함.
    
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
        
        beta        % h(x) 증가속도 (논문 Property III.2)
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
            
            obj.sigma_ref_prev = init_sigma_rad; 
            obj.V_p = V_p;
            obj.V_t = V_t;
            
            % CBF 튜닝 변수 (Property III.2의 0 < beta < 1)
            obj.beta = 0.01; % 으헤~ 일단 0.5로 둘게. 원하면 바꿔!
        end
        
        %% Helper: h(x) 계산기 (위험도 평가)
        function h_val = get_hx(obj, sig_p_cand)
            epsilon = 1e-7;
            C = (2 * obj.r_allow * obj.max_acc) / (obj.V_p^2);
            
            % sigma_t 범위를 0부터 pi까지 생성 (maxAcc_BRS_Solution 기준)
            sigma_t_sweep = linspace(0, pi, 1000); 
            
            % 좌변 함수 계산
            y = (sin(sigma_t_sweep) - sin(sig_p_cand)) .* (1 + cos(sigma_t_sweep + sig_p_cand)) ./ (cos(sig_p_cand).^2 + epsilon);
            
            % 교점 찾기
            diff_y = y - C;
            cross_idx = find(diff_y(1:end-1) .* diff_y(2:end) <= 0);
            
            if length(cross_idx) >= 2
                % Step 2. 겹치는 부분(해)이 존재하면 두 교점을 구함
                p1 = sigma_t_sweep(cross_idx(1));
                p2 = sigma_t_sweep(cross_idx(end));
                L = abs(p2 - p1);
                h_val = -L; % 길이에 음수를 붙여서 위험도 표시
            else
                % 겹치지 않으면 안전
                h_val = 0;  
            end
        end
        
        %% 메인 유도 명령 계산 함수
        function [acc_cmd, sigma_ref_curr, V_clf, H_cbf, slack_opt] = compute_commandRDPG_SAFE(obj, X_State, lambda_dot)
            
            r = X_State(1);
            sigma_t = X_State(2);
            sigma_p = X_State(3);
            
            % [기본 RDPG 로직] 최적 리드각 계산
            eqn = @(x) sqrt(r/obj.r_allow) * cos((sigma_t + x)/2) - cos(x);
            try
                nom_sigma_ref = fzero(eqn, obj.sigma_ref_prev);
            catch
                nom_sigma_ref = obj.sigma_ref_prev;
            end
            
            % 기본 DPG 명령
            nom_u_cmd = lambda_dot - obj.k * (sigma_p - nom_sigma_ref);
            nom_acc = max(min(obj.V_p * nom_u_cmd, obj.max_acc), -obj.max_acc);
            
            % Step 1 & 2. 현재 sigma_p 기반으로 h(x) 평가
            h_curr = obj.get_hx(sigma_p);
            
            % 로깅용 변수 세팅
            H_cbf = h_curr;
            V_clf = (sigma_p - nom_sigma_ref)^2;
            
            if h_curr == 0
                % 해가 존재하지 않으면 Safe! RDPG 제어 그대로 사용
                acc_cmd = nom_acc;
                sigma_ref_curr = nom_sigma_ref;
                slack_opt = 0;
            else
                % Step 3 & 4. 위험지역! h(x)를 안전하게 만드는 제어 입력 탐색
                % 예측 기반으로 가능한 가속도 후보군을 스윕
                a_cands = linspace(-obj.max_acc, obj.max_acc, 41);
                h_next_vals = zeros(size(a_cands));
                valid_idx = [];
                
                for i = 1:length(a_cands)
                    % 해당 가속도 인가 시 예측되는 다음 sigma_p
                    sig_p_next = sigma_p + (a_cands(i)/obj.V_p - lambda_dot) * obj.dt;
                    h_next_vals(i) = obj.get_hx(sig_p_next);
                    
                    % CBF 조건 검사: h(x_next) - h(x_curr) >= -beta * h(x_curr)
                    if h_next_vals(i) >= (1 - obj.beta) * h_curr
                        valid_idx(end+1) = i;
                    end
                end
                
                if ~isempty(valid_idx)
                    % 조건을 만족하는 해가 여러 개면 원래 의도한 제어(nom_acc)와 가장 가까운 것 선택
                    [~, min_diff_idx] = min(abs(a_cands(valid_idx) - nom_acc));
                    acc_cmd = a_cands(valid_idx(min_diff_idx));
                    slack_opt = 0; 
                else
                    % 조건을 아예 만족하는 해가 없다면, h(x)를 가장 덜 위험하게(0에 가깝게) 만드는 후보 채택
                    [max_h, max_idx] = max(h_next_vals);
                    acc_cmd = a_cands(max_idx);
                    slack_opt = (1 - obj.beta) * h_curr - max_h; % 위반량 기록
                end
                
                sigma_ref_curr = nom_sigma_ref; 
            end
            
            obj.sigma_ref_prev = sigma_ref_curr;
        end
    end
end