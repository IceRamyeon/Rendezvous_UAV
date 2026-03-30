%% main.m
close all; 
clc; 
clear;

addpath('./Rendezvous_Simulation/Guidance')
addpath('./Rendezvous_Simulation/Inital_Conditions')
addpath('./Rendezvous_Simulation/Plotting_function')
addpath('./Rendezvous_Simulation/BRS&max_acc')

%% 0. 시나리오 입력 설정 (으헤~ 여기만 바꾸면 돼!)
% [모드 선택]
% 'REG' : 도달 가능 영역 분석 (CSV 저장)
% 'SIM' : 1:1 궤적 시뮬레이션
RUN_MODE = 'SIM';

% [초기 위치 입력 설정] (SIM 모드에서만 사용)
input_type = 0;             % 0: 극좌표계 (a = 거리, b = 베어링각) / 1: 직교좌표계 (a = X, b = Y)
input_a = 1000;             % input_type 0일 땐 거리[m], 1일 땐 X[m]
input_b = -50;              % input_type 0일 땐 베어링[deg], 1일 땐 Y[m]

% [초기 리드각 설정]
% 직접 입력하려면 숫자를 넣고 (예: 60), 자동으로 RDPG 해를 찾게 하려면 빈 대괄호 [] 로 둬.
sigma_p0_deg = [];          

%% 1. 공통 파라미터 설정
% [유도 법칙 설정]
GUIDANCE_Switch = 4;    % 1 = PPNG, 2 = DPG, 3 = VDPG, 4 = RDPG, 5 = RDPG_T, 6 = RDPG_SAFE
switch GUIDANCE_Switch
    case 1, GUIDANCE_MODE = 'PPNG';
    case 2, GUIDANCE_MODE = 'DPG';
    case 3, GUIDANCE_MODE = 'VDPG';
    case 4, GUIDANCE_MODE = 'RDPG';
    case 5, GUIDANCE_MODE = 'RDPG_T';
    case 6, GUIDANCE_MODE = 'RDPG_SAFE';
end
cfg.GUIDANCE_MODE = GUIDANCE_MODE;
cfg.V_p = 20.0;                 % [m/s] Pursuer 속도
cfg.V_t = 20.0;                 % [m/s] Target 속도
target_turn_rate_deg = 0.0;     % [deg/s] 탱커의 기본 회전율
cfg.At_constant = cfg.V_t * deg2rad(target_turn_rate_deg);

% [시뮬레이션 시간 설정]
cfg.dt_region = 0.01;           % Time Step for region sweep
cfg.dt_simul = 0.01;            % Time Step for Simulation
cfg.tf = 60;                    % Final Time
cfg.pause_t = 0.1;              % Pause Time
cfg.skip_frame = 70;            % Animation 속도 조절

% Stop Condition / auto save 사용여부
cfg.stop_condition = 0;         % 0 = off, 1 = on, 단 RDPG_T는 무조건 on
cfg.auto_save = 0;

% 저장 경로 설정 (선생의 노트북 환경에 맞게 수정!)
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260327 세미나 준비'; 
cfg.save_dir = fullfile(target_path, 'bearing = 64도,RDPG_SAFE');

%% 2. 유도 법칙별 파라미터 통합 설정
cfg.limit_acc = 1;              % [G] 가속도 제한
cfg.N = 3.0;                    % Navigation Constant (PPNG)
cfg.bias_acc = -0.0;            % Bias Acceleration
cfg.gain_k = 5.0;               % 유도 게인 K (DPG, VDPG, RDPG 공용)

% [Success Criteria + RDPG, RDPG_T 파라미터]
cfg.r_allow = 2;                % [m] 허용 거리
cfg.r_theory_m = cfg.r_allow;
cfg.th_psi_deg = 5.0;           % [deg] 성공 헤딩 오차
cfg.th_lead_deg = 5.0;          % [deg] 성공 lead angle 오차
rate_limit_degree = 15;         % 최대 시선각 변화율
cfg.rate_limit = deg2rad(rate_limit_degree);
cfg.alpha = 1;                  % LPF 계수

% RDPG_T 파라미터
target_turn_rate_cmd_deg = 0.4;
cfg.At_cmd = cfg.V_t * deg2rad(target_turn_rate_cmd_deg);
cfg.RDPG_T_MODE = 1;            % 0 = lead angle, 1 = relative distance
cfg.sigma_offset = 9;           % lead angle 모드일 때 offset
cfg.RDPG_test = 1;              % test mode on = 1, off = 0

%% 3. 모드별 추가 설정 및 실행
switch RUN_MODE
    case 'REG'
        % --- Region 모드 전용 설정 ---
        cfg.r_min_km = 0.001; cfg.r_max_km = 0.001; cfg.r_step_km = 0.1;
        cfg.b_min_deg = -0; cfg.b_max_deg = -0; cfg.b_step_deg = 0.0;
        cfg.sigma_cases_deg = [0, 15, 30, 45, 60, 75];

        fprintf('>>> [Reachability Region : "%s"]\n', cfg.GUIDANCE_MODE);
        Randezvous_Region(cfg);
        
    case 'SIM'
        % [Target 초기 위치 (Global)]
        cfg.Xt_input_km = 0.2;       % [Km] 타겟 Global X
        cfg.Yt_input_km = 0.0;       % [Km] 타겟 Global Y
        cfg.psi_ti_deg = 90;         % [deg] 타겟 초기 헤딩

        % [Pursuer 초기 상대 위치 계산]
        if input_type == 0
            cfg.r_from_region_m = input_a;          % [m] 초기 거리
            cfg.bearing_from_region = input_b;      % [deg] 초기 베어링 (Target 기준)
        elseif input_type == 1
            r = sqrt(input_a^2 + input_b^2);
            theta_raw = atan(input_b/input_a);
            theta_deg = -rad2deg(pi/2 + theta_raw);
            cfg.r_from_region_m = r;
            cfg.bearing_from_region = theta_deg;
        else
            error('으헤... input_type은 0 아니면 1이어야 해!');
        end

        % [초기 리드각(sigma_p0) 설정 및 계산]
        if isempty(sigma_p0_deg)
            % DPG_lead 입력이 없을 때: RDPG 초기화 로직(fzero) 수행
            bearing_rad = cfg.bearing_from_region * pi / 180;
            sigma_t_init_rad = bearing_rad - pi; 
            sigma_t_init_rad = atan2(sin(sigma_t_init_rad), cos(sigma_t_init_rad));
            
            eqn = @(x) sqrt(cfg.r_from_region_m / cfg.r_allow) * cos((sigma_t_init_rad + x)/2) - cos(x);
            
            try
                init_try_rad = 100 * pi / 180; 
                lead_p_init_rad = fzero(eqn, init_try_rad); 
                lead_p_init = lead_p_init_rad * 180 / pi; 
            catch
                disp('으헤... 초기 sigma_ref 해를 찾지 못했어. 일단 0으로 둘게.');
                lead_p_init = 0; 
            end   
        else
            % 입력된 값이 있을 때 그대로 사용
            lead_p_init = sigma_p0_deg;
        end
        
        % 초기 헤딩 각도 최종 적용
        cfg.psi_p_from_region = lead_p_init - cfg.bearing_from_region - 90;

        fprintf('>>> [rate_limit: %.1f]\n', cfg.rate_limit);
        fprintf('>>> [Simulation 모드: r=%.1f, b=%.1f, 초기 리드각=%.1f]\n', cfg.r_from_region_m, cfg.bearing_from_region, lead_p_init);
        Randezvous_Simulation(cfg);
        
    otherwise
        error('모드 없음.');
end