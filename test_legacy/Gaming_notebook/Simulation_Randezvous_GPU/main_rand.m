%% main_rand.m (GPU Version)
clear; clc; close all;

%% 1. Configuration and Parameters
cfg = struct();

% Simulation Time
cfg.dt_region = 0.05;    % 시간 간격 (GPU는 빠르니까 좀 더 줄여도 됨)
cfg.tf = 300;            % 최대 시뮬레이션 시간 [sec]

% Vehicle Specifications
cfg.V_p = 20;            % Pursuer Speed [m/s]
cfg.V_t = 20;            % Target Speed [m/s]
cfg.At_cmd = 0;          % Target Acceleration
cfg.limit_acc = 2.0;     % Acceleration Limit [g] (중요!)

% Guidance Parameters (RDPG)
cfg.GUIDANCE_MODE = 'RDPG';
cfg.RDPG.K = 3.0;        % Guidance Gain (논문 또는 기존 코드 값 참조)
cfg.RDPG.sigma_d = 0;    % Desired Look Angle [deg]

% Success Criteria
cfg.r_allow = 5.0;       % Capture Distance [m]
cfg.th_psi_deg = 30.0;   % Heading Alignment Threshold [deg]

% Initial Condition Sweep Range (Meshgrid 생성용)
% 기존 코드보다 범위를 넓히거나 step을 촘촘하게 해도 GPU라 금방 끝남
cfg.r_min_km = 0.1;      
cfg.r_max_km = 1.0;      
cfg.r_step_km = 0.005;    % 50m 간격 (촘촘하게)

cfg.b_min_deg = -180;       
cfg.b_max_deg = -0;     
cfg.b_step_deg = 1.0;    % 1도 간격

% Simulation Cases (초기 Look Angle)
cfg.sigma_cases_deg = [0, 10, 20, 40]; 

%% 2. Run GPU Accelerated Simulation
fprintf('========================================\n');
fprintf('   Reachability Simulation (GPU Mode)   \n');
fprintf('========================================\n');

try
    tic;
    Randezvous_Region_GPU(cfg);
    elapsed_time = toc;
    fprintf('\n>>> 전체 시뮬레이션 완료! 소요 시간: %.2f초 <<<\n', elapsed_time);
catch ME
    fprintf('\n!!! 오류 발생 !!!\n%s\n', ME.message);
    % GPU 메모리 부족 시 조치 팁 출력
    if contains(ME.message, 'Out of memory')
        fprintf('팁: cfg.r_step_km 또는 cfg.b_step_deg 값을 조금 키워서 점의 개수를 줄여보세요.\n');
    end
end