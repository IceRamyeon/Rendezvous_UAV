function main_Rendezvous(a, b, input_type, sigma_p0_deg)
%% main_rand.m
% 함수형으로 변환: main_rand(a, b, input_type)
% input_type = 0: a = r, b = bearing (극좌표계)
% input_type = 1: a = x, b = y (직교좌표계, car2pol 사용)
% 입력이 없으면(nargin == 0) 자동으로 RUN_MODE = 'REG' 실행

close all; 
clc; 
% clear; % 함수 내에서 clear를 사용하면 입력받은 a, b, input_type이 날아가므로 주석 처리

%% 1. 공통 파라미터 설정
% [모드 선택]
% 'REG' : 도달 가능 영역 분석 (CSV 저장)
% 'SIM'    : 1:1 궤적 시뮬레이션

if nargin == 0
    RUN_MODE = 'REG';
else
    RUN_MODE = 'SIM';
end

% [유도 법칙 설정]
GUIDANCE_Switch = 2;    % 1 = PPNG, 2 = DPG, 3 = VDPG, 4 = RDPG, 5 = RDPG_T, 6 = RDPG_SAFE
    switch GUIDANCE_Switch
        case 1
            GUIDANCE_MODE = 'PPNG';
        case 2
            GUIDANCE_MODE = 'DPG';
        case 3
            GUIDANCE_MODE = 'VDPG';
        case 4
            GUIDANCE_MODE = 'RDPG';
        case 5
            GUIDANCE_MODE = 'RDPG_T';
        case 6
            GUIDANCE_MODE = 'RDPG_SAFE';
    end
cfg.GUIDANCE_MODE = GUIDANCE_MODE;
cfg.V_p = 20.0;                 % [m/s] Pursuer 속도
cfg.V_t = 20.0;                 % [m/s] Target 속도
target_turn_rate_deg = 0.0;   % [deg/s] 탱커의 기본 회전율
cfg.At_constant = cfg.V_t * deg2rad(target_turn_rate_deg);

% [시뮬레이션 시간 설정]
cfg.dt_region = 0.01;           % Time Step for region sweep
cfg.dt_simul = 0.01;            % Time Step for Simulation
cfg.tf = 60;                    % Final Time
cfg.pause_t = 0.1;              % Pause Time
cfg.skip_frame = 70;            % Animation 속도 조절, auto_save가 켜지면 느려지므로 Frame을 70 정도로 잡을 것.

% Stop Condition / auto save 사용여부
cfg.stop_condition = 0;         % 0 = off, 1 = on, 단 RDPG_T는 무조건 on
cfg.auto_save = 1;
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260327 세미나 준비'; 
cfg.save_dir = fullfile(target_path, 'bearing = 64도,RDPG_SAFE');

%% 2. 유도 법칙별 파라미터 통합 설정

% [Common] 가속도 제한
cfg.limit_acc = 1;         % [G]

% [PPNG 파라미터]
cfg.N = 3.0;                % Navigation Constant
cfg.bias_acc = -0.0;        % Bias Acceleration

% [DPG / VDPG / RDPG 파라미터]
% cfg.target_lead_angle_deg = 60.0; % 목표 리드각 (DPG용)
cfg.gain_k = 5.0;                 % 유도 게인 K (DPG, VDPG, RDPG 공용)

% [Success Criteria + RDPG, RDPG_T 파라미터]
cfg.r_allow = 2;                  % [m] 허용 거리
cfg.r_theory_m = cfg.r_allow;
cfg.th_psi_deg = 5.0;      % [deg] 성공 헤딩 오차
cfg.th_lead_deg = 5.0;     % [deg] 성공 lead angle 오차
rate_limit_degree = 15;      % 최대 시선각 변화율
cfg.rate_limit = deg2rad(rate_limit_degree);
cfg.alpha = 1;      % LPF 계수

% RDPG_T 파라미터
target_turn_rate_cmd_deg = 0.4;
cfg.At_cmd = cfg.V_t * deg2rad(target_turn_rate_cmd_deg);
cfg.RDPG_T_MODE = 1;   % 0 = lead angle, 1 = relative distance
cfg.sigma_offset = 9; % lead angle 모드일 때 offset
cfg.RDPG_test = 1; % test mode on = 1, off = 0

%% 3. 모드별 추가 설정 및 실행
switch RUN_MODE
    case 'REG'
        % --- Region 모드 전용 설정 ---
        % (거리 Sweep)
        cfg.r_min_km = 0.001; cfg.r_max_km = 0.001; cfg.r_step_km = 0.1;
        
        % (Bearing Sweep)
        cfg.b_min_deg = -0; cfg.b_max_deg = -0; cfg.b_step_deg = 0.0;

        % (Sigma Case)
        cfg.sigma_cases_deg = [0, 15, 30, 45, 60, 75];
        % cfg.sigma_cases_deg = [0, 20, 40];

        % 실행!
        fprintf('>>> [Reachability Region : "%s"]\n', cfg.GUIDANCE_MODE);
        Randezvous_Region(cfg);
        
    case 'SIM'
        % [Target 초기 위치 (Global)]
        cfg.Xt_input_km = 0.2;       % [Km] 타겟 Global X
        cfg.Yt_input_km = 0.0;       % [Km] 타겟 Global Y
        cfg.psi_ti_deg = 90;        % [deg] 타겟 초기 헤딩

        % [Pursuer 초기 상대 위치 (입력값에 따른 분기 처리)]
        if input_type == 0
            cfg.r_from_region_m = a;          % [m] 초기 거리
            cfg.bearing_from_region = b;      % [deg] 초기 베어링 (Target 기준)
        elseif input_type == 1
            r = sqrt(a^2 + b^2);
            theta_raw = atan(b/a);
            theta_deg = -rad2deg(pi/2 + theta_raw);
            cfg.r_from_region_m = r;
            cfg.bearing_from_region = theta_deg;
        else
            error('으헤... input_type은 0 아니면 1이어야 해!');
        end


        if nargin < 4 || isempty(sigma_p0_deg)
            % DPG_lead 입력이 없을 때: RDPG 초기화 로직(fzero) 수행
            % 1. bearing을 라디안으로 변환 (pi/180 곱하기)
            bearing_rad = cfg.bearing_from_region * pi / 180;
            
            % 2. 초기 target lead angle (sigma_t) 계산 (라디안)
            sigma_t_init_rad = bearing_rad - pi; 
            sigma_t_init_rad = atan2(sin(sigma_t_init_rad), cos(sigma_t_init_rad));
            
            % 3. RDPG 수식
            eqn = @(x) sqrt(cfg.r_from_region_m / cfg.r_allow) * cos((sigma_t_init_rad + x)/2) - cos(x);
            
            try
                % 0.4 (라디안) 근처에서 해를 찾도록 초기값 설정
                init_try_rad = 100 * pi / 180; 
                lead_p_init_rad = fzero(eqn, init_try_rad); 
                
                % fzero로 찾은 라디안 값을 다시 deg 단위로 변환
                lead_p_init = lead_p_init_rad * 180 / pi; 
            catch
                disp('으헤... 초기 sigma_ref 해를 찾지 못했어. 일단 0으로 둘게.');
                lead_p_init = 0; 
            end   
        else
            % DPG_lead 입력이 있을 때: 입력된 값을 그대로 사용
            sigma_p0 = deg2rad(sigma_p0_deg);
            lead_p_init = sigma_p0;
        end
        
        % 이제 아래 수식을 거치면 초기 sigma_p가 lead_p_init과 정확히 일치하게 됨
        cfg.psi_p_from_region = lead_p_init - cfg.bearing_from_region - 90;      % [deg] Pursuer 초기 헤딩

        % Table
        % --Head on--
        % (650, -51), (550,-27), (441.41,-17.6), (350,-48)
        % Not Head-on
        % (650, -81), (50, -63), (450, -90)
        % Some Success Some not
        % (450, -69)

        fprintf('>>> [rate_limit: %.1f]\n', cfg.rate_limit);
        fprintf('>>> [Simulation 모드: r=%.1f, b=%.1f]\n', cfg.r_from_region_m, cfg.bearing_from_region);
        Randezvous_Simulation(cfg);
        
    otherwise
        error('모드 없음.');
end

end