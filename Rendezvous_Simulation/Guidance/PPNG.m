classdef PPNG < handle
    % PPNG 유도 클래스
    properties
        N           % PPNG용 유도 상수 (Effective Navigation Ratio)
        bias_acc
        max_acc     % 가속도 제한값 (m/s^2)
    end
    
    methods
        %% CONSTRUCTOR
        function obj = PPNG(N_gain, bias_acc, limit_G)
            % 생성자: N_gain, limit_G
            if nargin < 1, N_gain = 3; end      % 기본값 3
            if nargin < 2, bias_acc = 0; end
            if nargin < 3, limit_G = 20; end    % 기본값 20G
             
            obj.N = N_gain;
            obj.bias_acc = bias_acc;
            obj.max_acc = limit_G * 9.81; % G단위를 m/s^2로 변환
          end
        
        %% Method PPNG (Proportional Navigation)
        function acc_cmd = compute_commandPPNG(obj, V_c, lambda_dot)
            
            % [공식] a = N * V_c * lambda_dot + bias
            raw_acc = (obj.N * V_c * lambda_dot) + obj.bias_acc;
            
            % Saturation
            if abs(raw_acc) > obj.max_acc
                acc_cmd = obj.max_acc * sign(raw_acc);
            else
                acc_cmd = raw_acc;
            end
        end
    end
end