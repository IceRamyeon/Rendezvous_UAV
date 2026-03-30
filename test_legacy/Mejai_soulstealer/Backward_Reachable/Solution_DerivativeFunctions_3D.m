clear; clc; close all;

%% 1. 범위 설정 및 3차원 격자 생성
% sigma_t는 0도부터 180도까지
sigma_t_deg = linspace(0, 180, 100);
% sigma_p는 분모가 0이 되는 걸 막기 위해 89도까지만!
sigma_p_deg = linspace(0, 89, 100); 

% 3차원 계산을 위해 Meshgrid 생성
[Sigma_T_deg, Sigma_P_deg] = meshgrid(sigma_t_deg, sigma_p_deg);

% 삼각함수 계산을 위해 라디안으로 변환
Sigma_T = deg2rad(Sigma_T_deg);
Sigma_P = deg2rad(Sigma_P_deg);

%% 2. 함수 및 도함수 계산
% 분모 (공통)
denom = cos(Sigma_P).^2;

% 원함수 f 계산
f_val = (sin(Sigma_T) - sin(Sigma_P)) .* (1 - cos(Sigma_T + Sigma_P)) ./ denom;

% 일계도함수 f' 계산
term1_1 = cos(Sigma_T);
term1_2 = cos(2*Sigma_T + Sigma_P);
term1_3 = sin(Sigma_P) .* sin(Sigma_T + Sigma_P);
f_prime = (term1_1 - term1_2 - term1_3) ./ denom;

% 이계도함수 f'' 계산
term2_1 = -sin(Sigma_T);
term2_2 = 2 * sin(2*Sigma_T + Sigma_P);
term2_3 = sin(Sigma_P) .* cos(Sigma_T + Sigma_P);
f_double_prime = (term2_1 + term2_2 - term2_3) ./ denom;

%% 3. 3차원 그래프 시각화 (Surface Plot)

% --------------------------------------------------
% [새로운 창] 원함수 f 3D 그래프
% --------------------------------------------------
figure('Name', 'Original Function 3D Surface', 'Theme', 'light', 'Position', [100, 50, 800, 500]);
surf(Sigma_T_deg, Sigma_P_deg, f_val, 'EdgeColor', 'none');
hold on;
% 기준선: z=0 평면 
surf(Sigma_T_deg, Sigma_P_deg, zeros(size(f_val)), 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
title('원함수 f(\sigma_p; \sigma_t)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('\sigma_t [degree]', 'FontWeight', 'bold');
ylabel('\sigma_p [degree]', 'FontWeight', 'bold');
zlabel('f Value', 'FontWeight', 'bold');
colorbar;
view(-30, 30); % 보기 좋은 각도로 카메라 회전
zlim([-10, 1]); % <--- [추가됨] z축 스케일 제한!
grid on; hold off;

% --------------------------------------------------
% [기존 창] 미분 함수들 (일계도, 이계도) 3D 그래프
% --------------------------------------------------
figure('Name', 'Derivatives 3D Surface', 'Theme', 'light', 'Position', [100, 100, 1200, 500]);

% [왼쪽] 일계도함수 f' 3D 그래프
subplot(1, 2, 1);
surf(Sigma_T_deg, Sigma_P_deg, f_prime, 'EdgeColor', 'none');
hold on;
% 기준선: z=0 평면 
surf(Sigma_T_deg, Sigma_P_deg, zeros(size(f_prime)), 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
title('일계도함수 f''(\sigma_p; \sigma_t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('\sigma_t [degree]', 'FontWeight', 'bold');
ylabel('\sigma_p [degree]', 'FontWeight', 'bold');
zlabel('f'' Value', 'FontWeight', 'bold');
colorbar;
view(-30, 30);
zlim([-10, 1]); % <--- [추가됨] z축 스케일 제한!
grid on; hold off;

% [오른쪽] 이계도함수 f'' 3D 그래프
subplot(1, 2, 2);
surf(Sigma_T_deg, Sigma_P_deg, f_double_prime, 'EdgeColor', 'none');
hold on;
% 기준선: z=0 평면 
surf(Sigma_T_deg, Sigma_P_deg, zeros(size(f_double_prime)), 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
title('이계도함수 f''''(\sigma_p; \sigma_t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('\sigma_t [degree]', 'FontWeight', 'bold');
ylabel('\sigma_p [degree]', 'FontWeight', 'bold');
zlabel('f'''' Value', 'FontWeight', 'bold');
colorbar;
view(-30, 30);
zlim([-10, 1]); % <--- [추가됨] z축 스케일 제한!
grid on; hold off;