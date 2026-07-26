close all;
clc; clear;

%% 1. 파라미터 및 좌표 설정
R2D = 180/pi;
D2R = pi/180;

% --- Simulation Param ---
dt = 0.01;
tf = 40;
time = 0:dt:tf;
V = 300;                    % Speed (m/s)

% --- 화살표 표시 크기 설정 ---
arrow_scale = 170; % 화면에 보일 화살표 크기 (시뮬레이션 스케일에 맞춰 조정)

% --- [STEP 1] Target 초기 상태 ---
Xt_i = 0; 
Yt_i = -3000;
psi_ti = 90 * D2R;          % Target Heading (North)

% --- [STEP 2] Pursuer 초기 위치 정의 (Target 기준) ---
r_i = 3000;                 % 초기 거리
lambda_param = 70;          % 배치용 각도 파라미터 (deg)
theta_rel = (90 + lambda_param) * D2R; 

% Pursuer 위치 계산
Xp_i = Xt_i + r_i * cos(theta_rel);
Yp_i = Yt_i + r_i * sin(theta_rel);

% --- [STEP 3] 관성 좌표계 기준 각도(LOS) 역산 ---
Rx = Xt_i - Xp_i;
Ry = Yt_i - Yp_i;
lambda_inertial = atan2(Ry, Rx); 

% --- [STEP 4] 이등변 삼각형 원리를 이용한 최적 헤딩 자동 설정 ---

% 1. 상대 위치 벡터 (Target - Pursuer)
Rx = Xt_i - Xp_i;
Ry = Yt_i - Yp_i;
R_sq = Rx^2 + Ry^2; % 거리의 제곱

% 2. Target의 단위 방향 벡터 (Unit Vector)
% Target은 psi_ti 방향으로 등속 직선 운동함
Utx = cos(psi_ti);
Uty = sin(psi_ti);

% 3. 내적 (Dot Product): R dot Ut
% 의미: 타겟 진행 방향으로 우리가 얼마나 떨어져 있나?
dot_val = Rx * Utx + Ry * Uty;

% 4. 접선 거리 d 계산 (이등변 삼각형 공식)
% 공식: d = (R^2) / (2 * (R dot Ut))
% 유도: |P_meet - P_start|^2 = |P_meet - T_start|^2 조건에서 나옴
if abs(dot_val) < 1e-3
    error('으헤~ 타겟이랑 나란히 있어서 만날 수가 없는데? (내적 0)');
end

d_intercept = R_sq / (2 * dot_val);

% 5. 가능성 판별 및 헤딩 설정
if d_intercept < 0
    % d가 음수면 타겟 뒤쪽으로 가야 만난다는 뜻 (이미 지나침)
    % 속도가 같으면 절대 못 잡음.
    fprintf('으헤~ 비상! 타겟이 이미 너무 멀리 갔거나 내가 너무 앞서 있어.\n');
    fprintf('이 조건(Vp=Vt)으로는 물리적으로 랑데뷰 불가!\n');
    
    % (일단 시뮬레이션을 위해 그냥 타겟 보는 걸로 설정하지만 실패할 거임)
    psi_pi = atan2(Ry, Rx); 
else
    % 6. 접선 지점(Intercept Point) 좌표 계산
    X_meet = Xt_i + d_intercept * Utx;
    Y_meet = Yt_i + d_intercept * Uty;
    
    % 7. 그 지점을 향해 헤딩 설정
    psi_pi = atan2(Y_meet - Yp_i, X_meet - Xp_i);
    
    fprintf('★ 최적 경로 발견! 타겟이 %.1fm 이동했을 때 만납니다.\n', d_intercept);
    fprintf('   Pursuer는 %.1f도 방향으로 출발합니다.\n', psi_pi * R2D);
end

% 8. Sigma 및 초기값 재설정
sigma_ti = psi_ti - lambda_inertial; % 이건 그대로
sigma_pi = psi_pi - lambda_inertial; % 계산된 psi_pi 적용

% [중요] 속도가 같다면 Vp = Vt = V로 설정
V = 300; % Pursuer, Target 동일 속도

% --- State Vector 초기화 ---
x = [r_i; sigma_ti; sigma_pi; lambda_inertial];

%% 2. 데이터 저장소 및 초기화
num_steps = length(time);
hist_state = zeros(4, num_steps);
hist_u = zeros(1, num_steps);

current_Xp = Xp_i; current_Yp = Yp_i;
current_Xt = Xt_i; current_Yt = Yt_i;

%% ========================================================================
%  실시간 그래픽 초기 설정 (Patch 사용)
% * patch는 'Faces'와 'Vertices'로 다각형을 그립니다.
% * 초기에는 빈 데이터(nan)로 핸들만 생성해 둡니다.

% 1. Target Centered View
fig1 = figure('Position', [10 300 400 400], 'Name', '1. Target Centered (Real-time)');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Target Centered Frame');
% 궤적 선
h_traj_relT = animatedline('Color', 'b', 'LineWidth', 1.5);
% 화살표 (Target: Red, Pursuer: Blue)
% 기준이 되는 Target은 보기 편하게 북쪽(90도) 고정으로 표시합니다.
h_arrow_T_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', 'k'); 
h_arrow_P_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', 'k');

% 2. Pursuer Centered View
fig2 = figure('Position', [420 300 400 400], 'Name', '2. Pursuer Centered (Real-time)');
grid on; axis equal; hold on;
xlabel('Rel X [m]'); ylabel('Rel Y [m]'); title('Pursuer Centered Frame');
h_traj_relP = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
% 기준이 되는 Pursuer는 보기 편하게 북쪽(90도) 고정으로 표시합니다.
h_arrow_P_fixed = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', 'k');
h_arrow_T_rel   = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', 'k');

% 3. Inertial Frame View
fig3 = figure('Position', [830 300 400 400], 'Name', '3. Inertial Trajectory (Real-time)');
grid on; axis equal; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Inertial Frame');
h_traj_P_inertial = animatedline('Color', 'b', 'LineWidth', 1.5);
h_traj_T_inertial = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
% 실제 움직이는 화살표들
h_arrow_P_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'b', 'EdgeColor', 'k');
h_arrow_T_inertial = patch('Vertices', nan(4,2), 'Faces', [1 2 3 4], 'FaceColor', 'r', 'EdgeColor', 'k');
legend([h_traj_P_inertial, h_traj_T_inertial], {'Pursuer Path', 'Target Path'}, 'Location', 'Best');


%% 3. 제어 게인
N = 3; 
K_b = 5; 

%% 4. 시뮬레이션 루프

% [추가] 그래프 창 띄우고 잠시 대기
figure(fig1); figure(fig2); figure(fig3); % 창을 앞으로 가져옴
drawnow;
pause(0.1); % 대기 시간 (s)

for i = 1:num_steps
    % Unpack
    r = x(1);
    sigma_t = x(2);
    sigma_p = x(3);
    lambda = x(4);
    
    % Save History
    hist_state(:, i) = x;
    
    % Kinematics
    r_dot = V * (cos(sigma_t) - cos(sigma_p));
    V_c = -r_dot;
    lambda_dot = -(V / r) * (sin(sigma_t) - sin(sigma_p)); 
    
    t_go = r / (V_c + 1e-6);
    if t_go < 0, t_go = 0; end
    
    % Guidance Law
    sigma_error = sigma_p - sigma_t;
    if t_go > 0.5
        b = (K_b * sigma_error) / t_go;
    else
        b = 0;
    end
    
    % Input Calculation
    acc_cmd = N * V_c * lambda_dot + b * V_c;
    limit = 20 * 9.81;
    if abs(acc_cmd) > limit, acc_cmd = limit * sign(acc_cmd); end
    hist_u(i) = acc_cmd;
    
    % Derivatives
    u_angular = acc_cmd / V; 
    d_r = r_dot;
    d_sigma_t = -lambda_dot;
    d_sigma_p = u_angular - lambda_dot;
    d_lambda = lambda_dot;
    
    x_dot = [d_r; d_sigma_t; d_sigma_p; d_lambda];
    
    % Update State
    x = x + x_dot * dt;
    
    % Update Position & Heading (Inertial)
    psi_p = x(3) + x(4); % 현재 Pursuer Heading
    psi_t = x(2) + x(4); % 현재 Target Heading
    
    current_Xp = current_Xp + V * cos(psi_p) * dt;
    current_Yp = current_Yp + V * sin(psi_p) * dt;
    current_Xt = current_Xt + V * cos(psi_t) * dt;
    current_Yt = current_Yt + V * sin(psi_t) * dt;

    %% ====================================================================
    %% [NEW] 실시간 화살표 업데이트 (Loop 내부)
    %% ====================================================================
    
    % 1. 상대 좌표 계산
    relX_T = current_Xp - current_Xt; relY_T = current_Yp - current_Yt; % Target 기준 P 위치
    relX_P = current_Xt - current_Xp; relY_P = current_Yt - current_Yp; % Pursuer 기준 T 위치
    
    % 2. Figure 1 업데이트 (Target Centered)
    addpoints(h_traj_relT, relX_T, relY_T);
    % Target은 (0,0)에 북쪽(90도) 고정 표시
    V_T_fixed = get_arrow_vertices(0, 0, 90*D2R, arrow_scale);
    % Pursuer는 상대 위치에 실제 헤딩(psi_p)으로 표시
    V_P_rel   = get_arrow_vertices(relX_T, relY_T, psi_p, arrow_scale);
    set(h_arrow_T_fixed, 'Vertices', V_T_fixed);
    set(h_arrow_P_rel, 'Vertices', V_P_rel);
    
    % 3. Figure 2 업데이트 (Pursuer Centered)
    addpoints(h_traj_relP, relX_P, relY_P);
    % Pursuer는 (0,0)에 북쪽(90도) 고정 표시
    V_P_fixed = get_arrow_vertices(0, 0, 90*D2R, arrow_scale);
    % Target은 상대 위치에 실제 헤딩(psi_t)으로 표시
    V_T_rel   = get_arrow_vertices(relX_P, relY_P, psi_t, arrow_scale);
    set(h_arrow_P_fixed, 'Vertices', V_P_fixed);
    set(h_arrow_T_rel, 'Vertices', V_T_rel);
    
    % 4. Figure 3 업데이트 (Inertial Frame - 실제 움직임)
    addpoints(h_traj_P_inertial, current_Xp, current_Yp);
    addpoints(h_traj_T_inertial, current_Xt, current_Yt);
    % 둘 다 실제 위치와 실제 헤딩으로 표시
    V_P_inertial = get_arrow_vertices(current_Xp, current_Yp, psi_p, arrow_scale);
    V_T_inertial = get_arrow_vertices(current_Xt, current_Yt, psi_t, arrow_scale);
    set(h_arrow_P_inertial, 'Vertices', V_P_inertial);
    set(h_arrow_T_inertial, 'Vertices', V_T_inertial);
    
    % 5. 화면 갱신
    drawnow limitrate; 
    
    %% ====================================================================

    % Stop Condition
    if r < 5
        hist_state = hist_state(:, 1:i);
        hist_u = hist_u(1:i);
        time = time(1:i);
        fprintf('랑데뷰 성공 t=%.2f\n', time(end));
        break;
    end
end

%% 5. 상태 변수 그래프
figure('Position', [10 50 1200 350], 'Name', 'State Variables');
subplot(1,3,1); plot(time, hist_state(1,:), 'g-', 'LineWidth', 2); grid on; 
title('Range (r)'); xlabel('Time [s]'); ylabel('m');
subplot(1,3,2); plot(time, hist_state(2,:)*R2D, 'r-', 'LineWidth', 2); grid on; 
title('\sigma_t (deg)'); xlabel('Time [s]');
subplot(1,3,3); plot(time, hist_state(3,:)*R2D, 'b-', 'LineWidth', 2); grid on; 
title('\sigma_p (deg)'); xlabel('Time [s]');

%% ========================================================================
%% [HELPER FUNCTION] 화살표 모양 꼭짓점 계산 함수
%% ========================================================================
function V_rot = get_arrow_vertices(x, y, psi, scale)
    % 1. 기본 화살표 모양 정의 (오른쪽(0도)을 가리키는 형상)
    % [x1 y1; x2 y2; ...] 정규화된 좌표
    V_base = [ 1.5,  0.0;   % 앞 코
              -1.0,  0.8;   % 왼쪽 날개 뒤
              -0.5,  0.0;   % 뒤쪽 중심 (약간 들어감)
              -1.0, -0.8];  % 오른쪽 날개 뒤
          
    % 스케일 적용
    V_base = V_base * scale;
    
    % 2. 회전 행렬 (Rotation Matrix)
    R = [cos(psi), -sin(psi);
         sin(psi),  cos(psi)];
     
    % 3. 회전 및 이동 변환
    % V_rot = (R * V_base')' + [x, y]
    V_rot = (R * V_base')'; % 회전 적용
    V_rot(:,1) = V_rot(:,1) + x; % X 이동
    V_rot(:,2) = V_rot(:,2) + y; % Y 이동
end