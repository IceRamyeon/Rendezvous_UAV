classdef RDPG_MAX < handle
    properties
        k, max_acc, r_allow, rate_limit, dt
        sigma_ref_prev
    end

    methods
        function obj = RDPG_MAX(k_gain, limit_G, r_allow, rate_limit, dt, init_sigma_rad)
            obj.k = k_gain;
            obj.max_acc = limit_G * 9.81;
            obj.rate_limit = rate_limit;
            obj.r_allow = r_allow;
            obj.dt = dt;
            obj.sigma_ref_prev = init_sigma_rad;
        end

    function [acc_cmd, ]
        % [Step 1] Object function f(\sigma_p;\sigma_t)를 수치해석적으로 계산하여 현재위치에서 랑데부가 가능한 r_0 lower bound를 계산한다.
        % 만약 r_0_lower bound > r_allow이면, 기존의 eqn = @(x) sqrt(r/obj.r_allow) * cos((sigma_t + x)/2) - cos(x); 를 사용하여 최적 리드각을 계산한다.
        % r_0_lower bound <= r_allow이면, 해당하는 sigma_p로 가도록 유도한다.


        % [Step 2] r_0_lower, r_0_upper 중 가까운 sigma_p선으로 가도록 Desired sigma_p를 유도. 
        
        % [Step 3] 변화율을 더하여 만약 (current_state - lower_state)*(current state - upper_state) < 0이 될 때까지 움직임.

        % [Step 4] DPG 명령 생성
        u_cmd = lambda_dot - obj.k * (sigma_p - sigma_ref_filtered);
        raw_acc = V_p * u_cmd;
        acc_cmd = max(min(raw_acc, obj.max_acc), -obj.max_acc);

    end
end