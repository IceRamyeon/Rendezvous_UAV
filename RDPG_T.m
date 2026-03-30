classdef RDPG_T < handle
    properties
        k, max_acc, r_allow, rate_limit, dt, mode
        sigma_ref_prev 
        alpha 
        sigma_offset
        tanker_on_prev 
        test_mode
        
        % [NEW] 안전 기동 졸업 여부 플래그
        safety_done 
    end
    
    methods
        function obj = RDPG_T(k_gain, limit_G, r_allow, rate_limit, dt, init_sigma_rad, mode, alpha, sigma_offset, test_mode)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.r_allow = r_allow;
            obj.rate_limit = rate_limit;
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
            obj.mode = mode;
            if nargin < 8, obj.alpha = 1.0; else, obj.alpha = alpha; end
            if nargin < 9, obj.sigma_offset = 0; else, obj.sigma_offset = sigma_offset; end
            if nargin < 10, obj.test_mode = 0; else, obj.test_mode = test_mode; end
            
            obj.tanker_on_prev = 1;
            obj.safety_done = false; 
        end

        function [acc_cmd, sigma_ref_filtered, tanker_turn_on] = compute_commandRDPG(obj, V_p, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop)
    
            % [유틸리티] 각도 정규화
            wrapToPi_custom = @(x) atan2(sin(x), cos(x));
            
            % 1. Bearing 계산
            sigma_t_wrapped = wrapToPi_custom(sigma_t);
            bearing_rad = abs(pi - abs(sigma_t_wrapped)); 
            bearing_deg = rad2deg(bearing_rad);
            
            % -----------------------------------------------------------
            % [Graduation Check] 졸업 요건
            % -----------------------------------------------------------
            if bearing_deg > 70
                obj.safety_done = true;
            end
            
            % -----------------------------------------------------------
            % [Logic Branch] Case 1 vs Case 2
            % -----------------------------------------------------------
            is_case2 = false;
            
            % [Case 2 Condition]: Test Mode 켜짐 AND 아직 졸업 안 함
            if obj.test_mode && (~obj.safety_done)
                is_case2 = true;
            end
            
            if is_case2
                % =======================================================
                % [Case 2 Logic] Bearing <= 60 (Safety Mode)
                % =======================================================
                
                % 1. Pursuer: Offset 유지
                offset_deg = 0;
                offset = deg2rad(offset_deg);
                
                target_val = sigma_t_wrapped + offset;
                x_candidate = wrapToPi_custom(target_val);
                
                % 2. Tanker: -psi_t 기동
                tanker_turn_val = -1; 
                
                % Case 2에서는 강제로 상태 업데이트 (히스테리시스 불필요)
                obj.tanker_on_prev = tanker_turn_val;
                
                is_safety_maneuver = true;
                
            else
                % =======================================================
                % [Case 1 Logic] Bearing > 60 (Normal / Graduated)
                % =======================================================
                
                % 1. Pursuer Logic
                eqn = @(x) sqrt(r/obj.r_allow) * cos((sigma_t + x)/2) - cos(x);
                try 
                    x_candidate = fzero(eqn, obj.sigma_ref_prev); 
                catch
                    x_candidate = obj.sigma_ref_prev; 
                end
        
                if obj.test_mode
                    x_candidate = min(x_candidate, sigma_t);
                end
                
                % 2. Tanker Logic: V_r 기준 제어 with Deadzone
                % [Deadzone Logic] 폭 1 (±0.5)
                % Threshold: -10
                % OFF 조건: r_dot < -10.5 (확실히 빠름)
                % ON  조건: r_dot > -9.5  (좀 느려짐)
                % Middle  : 현상 유지 (obj.tanker_on_prev)
                
                threshold = -0;
                deadzone_half = 0.5; % 전체 폭 1
                
                if r_dot < (threshold - deadzone_half)       % < -10.5
                    tanker_turn_val = 0; % Stop Turning
                    obj.tanker_on_prev = 0; % 상태 저장
                    
                elseif r_dot > (threshold + deadzone_half)   % > -9.5
                    tanker_turn_val = 1; % Normal Turning
                    obj.tanker_on_prev = 1; % 상태 저장
                    
                else
                    % Deadzone (-10.5 ~ -9.5) -> 이전 상태 유지
                    % 으헤~ 여기가 중요해. 여기서 안 바꾸고 버티는 거야.
                    tanker_turn_val = obj.tanker_on_prev;
                end
                
                is_safety_maneuver = false;
            end
            
            % -----------------------------------------------------------
            % [Command Generation]
            % -----------------------------------------------------------
            if is_safety_maneuver
                sigma_ref_new = x_candidate;
                obj.sigma_ref_prev = sigma_ref_new; 
                sigma_ref_filtered = sigma_ref_new;
            else
                delta = wrapToPi_custom(x_candidate - obj.sigma_ref_prev);
                max_change = obj.rate_limit * obj.dt;
                if abs(delta) > max_change
                    delta = sign(delta) * max_change;
                end
                
                pred_val = obj.sigma_ref_prev + delta;
                diff_for_lpf = wrapToPi_custom(pred_val - obj.sigma_ref_prev);
                sigma_ref_filtered = obj.sigma_ref_prev + obj.alpha * diff_for_lpf;
                
                sigma_ref_filtered = wrapToPi_custom(sigma_ref_filtered);
                obj.sigma_ref_prev = sigma_ref_filtered;
            end
        
            % -----------------------------------------------------------
            % [Output]
            % -----------------------------------------------------------
            if is_stop
                tanker_turn_on = 0;
            else
                tanker_turn_on = tanker_turn_val;
            end
        
            sigma_err = wrapToPi_custom(sigma_p - sigma_ref_filtered);
            u_cmd = lambda_dot - obj.k * sigma_err;
            raw_acc = V_p * u_cmd;
            acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);
        end
    end
end