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
        % [Step 1]object function f(\sigma_p;\sigma_t)를 수치해석적으로 계산하여 Safe Rendezvous가 가능한 sigma_p를 계산한다.

        % [Step 2] 변화율 제한
        % [Step 3] DPG 명령 생성


    end
end