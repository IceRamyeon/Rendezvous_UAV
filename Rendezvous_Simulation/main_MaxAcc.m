%% main_MaxAcc.m
close all; 
clc; 
clear;

% [중요] 경로 설정에 새로 정의한 Helper_Function 디렉토리를 추가했어!
addpath('./Rendezvous_Simulation/Guidance')
addpath('./Rendezvous_Simulation/Initial_Conditions')
addpath('./Rendezvous_Simulation/Plotting_function')
addpath('./Rendezvous_Simulation/Helper_Function')

%% 1. 공통 시나리오 및 기하학 파라미터 설정
cfg.GUIDANCE_MODE = 'RDPG_ACC'; % DPG, RDPG, RDPG_ACC
cfg.V_p = 20.0;        
cfg.V_t = 20.0;
cfg.At_constant = 0.0;     

% 시뮬레이션 및 애니메이션 타임 파라미터
cfg.dt_simul = 0.01;       
cfg.tf = 80;               
cfg.pause_t = 0.005;        
cfg.skip_frame = 50;       
cfg.stop_condition = 0;    
cfg.auto_save = 0;         

% [제어 및 물리적 제한 가속도] 
cfg.limit_acc = 1;         % 1G 제한 상태에서 클리핑 거동 확인!
cfg.gain_k = 5.0; 
cfg.r_allow = 2.0;         
cfg.th_psi_deg = 5.0;      

% [초기 조건 입력] 
input_a = 800;             
input_b = -30;           % 초기 베어링이 65.15도 일 때 sigma_pc = 62.5도 근처로 조정됨.

cfg.r_from_region_m = input_a;
cfg.bearing_from_region = input_b;
cfg.Xt_input_km = 0.2;     
cfg.Yt_input_km = 0.0;
cfg.psi_ti_deg = 90;

% 저장 경로 설정
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260629 랩미팅 준비\2'; 
cfg.save_dir = fullfile(target_path, 'Theorem1_Verification_1G_Clip');

%% 2. Reachable한 초기 리드각 (sigma_p0) 역산 (Helper 함수 호출)

RDPG_FLAG = 0;
manual_sigma_deg = 60;
manual_sigma_rad = deg2rad(manual_sigma_deg);

if RDPG_FLAG
    % 자동 계산 로직
    [sigma_p0_deg, sigma_p0_rad] = RDPG_LeadAngle(input_a, input_b, cfg.r_allow);
    cfg.target_lead_angle_deg = sigma_p0_deg;
    cfg.psi_p_from_region = sigma_p0_deg - input_b - 90;
else
    % 수동 설정 로직
    sigma_p0_deg = manual_sigma_deg;
    sigma_p0_rad = manual_sigma_rad;
    cfg.target_lead_angle_deg = sigma_p0_deg;
    cfg.psi_p_from_region = sigma_p0_deg - input_b - 90;
end
%% 3. [Theorem 1] 수식 직접 계산 기법 (Helper 함수 호출)
theory = Max_Point(sigma_p0_rad, cfg.r_allow, cfg.V_p);

fprintf('======================================================\n');
fprintf('    [LHS 수식 직접 Grid 계산 기반 임계점 예측 완료]    \n');
fprintf('======================================================\n');
fprintf('1. 고정 리드각 (sigma_p0)     : %.4f deg\n', sigma_p0_deg);
fprintf('2. 예측된 임계 타겟각 (sigma_t*): %.4f deg\n', theory.sigma_t_star_deg);
fprintf('3. 예측된 임계 거리 (r*)       : %.4f m\n', theory.r_star);
fprintf('4. 예측된 최대 요구가속도 (a*) : %.4f m/s^2\n', theory.a_star);
fprintf('======================================================\n');

%% 4. 검증 시뮬레이션 실행 및 결과 회수
sim_results = Simulation_MaxAcc(cfg, theory);

%% 5. 대화창(Command Window) 최종 수치 매칭 결과 출력
fprintf('\n======================================================\n');
fprintf('    [Theorem 1 검증 완료: 수식 예측값 vs 시뮬레이션 실측값]    \n');
fprintf('======================================================\n');
fprintf(' [1] 최대 요구 가속도 크기 (Maximum Acceleration)\n');
fprintf('  - 예측값 (Predicted) : %.4f m/s^2  (%.2f G)\n', theory.a_star, theory.a_star / 9.81);
fprintf('  - 실측값 (Simulated) : %.4f m/s^2  (%.2f G)\n', sim_results.a_sim_at_max, sim_results.a_sim_at_max / 9.81);
fprintf('  - 절대오차 (Abs Error): %.6f m/s^2\n', abs(theory.a_star - sim_results.a_sim_at_max));
fprintf('------------------------------------------------------\n');
fprintf(' [2] 임계 상대 거리 지점 (Critical Distance r*)\n');
fprintf('  - 예측값 (Predicted) : %.4f m\n', theory.r_star);
fprintf('  - 실측값 (Simulated) : %.4f m\n', sim_results.r_sim_at_max);
fprintf('  - 절대오차 (Abs Error): %.6f m\n', abs(theory.r_star - sim_results.r_sim_at_max));
fprintf('------------------------------------------------------\n');
fprintf(' [3] 임계 타겟 리드각 지점 (Critical Target Lead Angle \\sigma_t*)\n');
fprintf('  - 예측값 (Predicted) : %.4f deg\n', theory.sigma_t_star_deg);
fprintf('  - 실측값 (Simulated) : %.4f deg\n', sim_results.sigma_t_sim_at_max);
fprintf('  - 절대오차 (Abs Error): %.6f deg\n', abs(theory.sigma_t_star_deg - sim_results.sigma_t_sim_at_max));
fprintf('======================================================\n');