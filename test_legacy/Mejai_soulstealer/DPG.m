classdef DPG < handle
    % DPG (Deviated Pursuit Guidance)
    % 목표: 일정한 리드각(sigma_des)을 유지하며 비행
    % 제어 법칙: u = lambda_dot - k * (sigma_p - sigma_des)
    
    properties
        k           % 피드백 게인
        max_acc     % 가속도 제한 (m/s^2)
        sigma_des   % 목표 리드각 (Rad 단위로 저장)
    end
    
    methods
        %% CONSTRUCTOR
        % 초기화: (목표 리드각[deg], 게인 K, 가속도 제한[G])
        function obj = DPG(sigma_des_deg, k_gain, limit_G)
            if nargin < 1, sigma_des_deg = 0.0; end
            if nargin < 2, k_gain = 3.0; end
            if nargin < 3, limit_G = 20; end
            
            obj.sigma_des = sigma_des_deg * (pi/180); % Deg -> Rad 변환 저장
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
        end

        %% Method: Compute Command
        % 필요한 인자: 속도(V_p), 시선각 변화율(lambda_dot), 현재 리드각(sigma_p)
        function acc_cmd = compute_commandDPG(obj, V_p, lambda_dot, sigma_p)
           
            % 1. 제어 입력 u (psi_dot) 계산
            % 오차(error) = (sigma_p - obj.sigma_des)
            u_cmd = lambda_dot - obj.k * (sigma_p - obj.sigma_des);
            
            % 2. 가속도 변환 (a_cmd = V_p * psi_dot)
            raw_acc = V_p * u_cmd;
            
            % 3. Saturation (G-Limit)
            if abs(raw_acc) > obj.max_acc
                acc_cmd = obj.max_acc * sign(raw_acc);
            else
                acc_cmd = raw_acc;
            end
        end
    end
end