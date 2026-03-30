classdef VDPG < handle
    % VDPG (Deviated Pursuit Guidance)
    % VDPG 수식: u = lambda_dot - k*(sigma_p - sigma_pc)
    
    properties
        N           % 유도 상수 (Nav Constant, 보통 3)
        k           % VDP용 피드백 게인 (논문 식 15의 k)
        max_acc     % 가속도 제한 (m/s^2)
    end
    
    methods
        %% CONSTRUCTOR
        function obj = VDPG(k_gain, limit_G)
            % 생성자: limit_G, k_gain
            if nargin < 1, k_gain = 2.0; end    % 튜닝 필요
            if nargin < 2, limit_G = 20; end
            
            obj.max_acc = limit_G * 9.81;
            obj.k = k_gain;
        end

        %% Method Deviated Pursuit Guidance
        % 필요한 인자: Pursuer 속도(V_p), 시선각 변화율, 현재 리드각, 현재 거리, 초기 거리, 초기 설정 각도
        function acc_cmd = compute_commandVDPG(obj, V_p, lambda_dot, sigma_p, r, r0, sigma_init)
            
            % 1. sigma_pc 계산 [수정됨]
            % (r / r0)를 곱해야 거리가 줄어들 때(r->0) 각도도 0이 됨
            sigma_pc = ((r0 -r) / r0) * sigma_init;
            
            % 2. u = psi_dot 계산
            u_cmd = lambda_dot - obj.k * (sigma_p - sigma_pc);
            
            % (나머지는 동일)
            raw_acc = V_p * u_cmd;
            
            if abs(raw_acc) > obj.max_acc
                acc_cmd = obj.max_acc * sign(raw_acc);
            else
                acc_cmd = raw_acc;
            end
        end
    end
end