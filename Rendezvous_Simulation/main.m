%% main.m 컨트롤 타워
close all; clc; clear;

% [중요] 경로 설정에 새로 정의한 Helper_Function 디렉토리를 추가했어!
addpath('./Rendezvous_Simulation/Guidance')
addpath('./Rendezvous_Simulation/Initial_Conditions')
addpath('./Rendezvous_Simulation/Plotting_function')
addpath('./Rendezvous_Simulation/Helper_Function')

%% 1. 초기조건 및 시뮬레이션 파라미터 설정
% RDPG : Reachability-based rendezvous guidance(2022)
% RDPG_ACC : RDPG + Acceleration Limits(Fail-Safe : DPG)
% RDPG_VT : RDPG + Acceleration Limits(Fail-Safe : Virtual Target)
% RDPG_r_f : RDPG + Acceleration Limits(Fail-safe : Use r_f_escape)
% RDPG_MIN : RDPG + Acceleration Limits(Fail-Safe : Use a_min)
% RDPG_FRS : RDPG + Accleration Limits(Fail-Safe : FRS, min deg change)
cfg.GUIDANCE_MODE = 'RDPG_FRS'; 

% [초기 조건 입력] 
% Pursuer 상대 위치 및 리드각
cfg.r_pursuer = 400;               % Target 기준 Pursuer의 초기 상대거리 [m]
cfg.theta_pursuer = -40;             % Target 기준 Pursuer의 초기 상대방위각 [deg]
RDPG_FLAG = 0;                  % 1 : 초기부터 Reachability based lead angle 사용, 0 : 수동으로 초기 리드각 설정
sigma_p0_deg = 0;              % RDPG_FLAG = 0이면 수동으로 초기 리드각 설정 가능. RDPG_FLAG = 1일 경우 sigma_p0_deg = [];
sigma_p0_rad = deg2rad(sigma_p0_deg);
cfg.rate_limit_deg = 30;                % Pursuer Lead Angle 수정 최대변화율

% Target 초기 위치 및 자세
cfg.Xt_input_km = 0.0;      % Inertial Frame Target의 x좌표[km]
cfg.Yt_input_km = 0.0;      % Inertial Frame Target의 y좌표[km]
cfg.psi_ti_deg = 90;        % Inertial Frame Target Heading Angle[deg]

% 시뮬레이션 결과 저장
cfg.auto_save = 0;          % Simulation 결과 자동저장 (1 : 자동 저장, 0 : 저장 안 함)
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260724 미팅준비\실험결과\0723_2'; % 저장 경로
cfg.save_dir = fullfile(target_path);
my_filename = sprintf('RDPG_MIN_%.1f', -cfg.theta_pursuer); % 최종 results들을 담은 파일 이름

% RDPG_VT 관련 파라미터
cfg.r_vt = 100.0;                           % Virtual Target의 거리[m]
theta_vt_deg = -75;                       % Virtual Target의 bearing angle[deg]
cfg.theta_vt_rad = deg2rad(theta_vt_deg);

% RDPG_MIN 관련 파라미터
cfg.a_min = 0.1;                        % Fail-Safe시 escape a_min [G]

% RDPG_r_f 관련 파라미터
cfg.r_f_escape = 200;                   % Fail-Safe시 escape r_f

% RDPG_FRS 관련 파라미터
cfg.t_for = 2;
cfg.sigma_pref_deg = 62.6;
cfg.sigma_FRS_list = -170: 10: 180;

% 시뮬레이션 및 애니메이션 타임 파라미터
cfg.dt_simul = 0.01;                    % Time step
cfg.tf = 50;                            % Total Simulation Time[s]
cfg.pause_t = 0.1;                      % 초기 Simulation이 로딩으로 인해 Jump하는 것을 막기 위한 시뮬레이션 지연
cfg.skip_frame = 50;                    % Simulation이 무거워지는 것을 막기 위해 Frame을 스킵함
cfg.stop_condition = 0;                 % 일정 조건을 만족할 경우 자동으로 Stop.(0: off, 1 : on)

% [제어 및 물리적 제한 가속도] 
cfg.limit_acc = 1;          % Max Acceleration [G]
cfg.gain_k = 5.0;           % DPG Gain Factor
cfg.r_allow = 2.0;          % r_f, 랑데부 성공에 판별할 거리 조건
cfg.th_psi_deg = 5.0;       % 랑데부 성공에 판별할 각도 조건
cfg.V_p = 20.0;             % Pursuer Velocity
cfg.V_t = 20.0;             % Target Velocity
cfg.At_constant = 0.0;      % Maneuvering Target constant turn(Reachability-Based rendezvousguidance(2022) etcetera simulation)

%% 2. Reachable한 초기 리드각 (sigma_p0) 역산 (Helper 함수 호출)

if RDPG_FLAG
    % 자동 계산 로직
    [sigma_p0_deg, sigma_p0_rad] = RDPG_LeadAngle(cfg.r_pursuer, cfg.theta_pursuer, cfg.r_allow);
    cfg.target_lead_angle_deg = sigma_p0_deg;
    cfg.psi_p_auto = sigma_p0_deg - cfg.theta_pursuer - 90;
else
    % 수동 설정 로직
    cfg.target_lead_angle_deg = sigma_p0_deg;
    cfg.psi_p_auto = sigma_p0_deg - cfg.theta_pursuer - 90;
end
%% 3. [Theorem 1] 수식 직접 계산 기법 (Helper 함수 호출)
theory = Max_Point(sigma_p0_rad, cfg.r_allow, cfg.V_p);

fprintf('======================================================\n');
fprintf('    [Max Acc 지점 예측]    \n');
fprintf('======================================================\n');
fprintf('1. 고정 리드각 (sigma_p0)     : %.4f deg\n', sigma_p0_deg);
fprintf('2. 예측된 임계 타겟각 (sigma_t*): %.4f deg\n', theory.sigma_t_star_deg);
fprintf('3. 예측된 임계 거리 (r*)       : %.4f m\n', theory.r_star);
fprintf('4. 예측된 최대 요구가속도 (a*) : %.4f m/s^2\n', theory.a_star);
fprintf('======================================================\n');

%% 4. 검증 시뮬레이션 실행 및 결과 회수
[sim_results, sim_out] = Simulation_main(cfg, theory);

%% 5. 대화창(Command Window) 최종 수치 매칭 결과 출력
fprintf('\n======================================================\n');
fprintf('    [Theorem 1 검증 : 수식 예측값 vs 시뮬레이션 실측값]    \n');
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

%% 6. [랑데부 완료] 최종 상태 출력
% sim_out.hist_state의 마지막 열(end)이 최종 상태를 의미해
r_i = sim_out.init.r_i;
bearing_i = sim_out.init.bearing_deg;
Xp_i = sim_out.init.Xp_i;
Yp_i = sim_out.init.Yp_i;

r_f = sim_out.hist_state(1, end);
sigma_p_f_deg = sim_out.hist_state(5, end) * (180/pi); % rad to deg
psi_p_f_deg = sim_out.hist_state(3, end) * (180/pi);   % rad to deg

fprintf('======================================================\n');
fprintf('  [시뮬레이션 완료]    \n');
fprintf('======================================================\n');
fprintf('1. Pursuer 초기 상대위치      : (%.1f m, %.1f deg)\n', r_i, bearing_i); 
fprintf('2. Pursuer 초기 Inertial 위치 : (%.1f m, %.1f m)\n', Xp_i, Yp_i); 
fprintf('------------------------------------------------------\n');
fprintf('3. 최종 랑데부 거리 (r_f)       : %.4f m\n', r_f);
fprintf('4. 최종 랑데부 리드각 (sigma_p) : %.4f deg\n', sigma_p_f_deg);
fprintf('5. 최종 랑데부 헤딩각 (psi_p)   : %.4f deg\n', psi_p_f_deg);
fprintf('======================================================\n');

% 패키징 및 저장 함수 호출
if cfg.auto_save
    Save_Log_Data(cfg.save_dir, my_filename, sim_out);
end