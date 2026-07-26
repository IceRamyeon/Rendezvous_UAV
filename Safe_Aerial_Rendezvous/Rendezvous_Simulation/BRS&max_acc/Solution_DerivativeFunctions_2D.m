clear; clc; close all;

%% 1. Inputs & Parameters
sigma_p_deg = 0;                 % [degree] 고정할 초기 각도 (선생 맘대로 변경 가능!)
sigma_p = deg2rad(sigma_p_deg);   % [rad]

n = 2000;                         % 곡선을 부드럽게 그리기 위한 점 개수
sigma_t_deg = linspace(0, 180, n);% sigma_t 범위: 0 ~ 180도
sigma_t = deg2rad(sigma_t_deg);   % [rad]

%% 2. Function Calculations (원함수, 1계도, 2계도)
% 공통 분모
denom = cos(sigma_p)^2;

% [1] 원함수 f
f_val = (sin(sigma_t) - sin(sigma_p)) .* (1 - cos(sigma_t + sigma_p)) / denom;

% [2] 일계도함수 f'
term1_1 = cos(sigma_t);
term1_2 = cos(2*sigma_t + sigma_p);
term1_3 = sin(sigma_p) .* sin(sigma_t + sigma_p);
f_prime = (term1_1 - term1_2 - term1_3) / denom;

% [3] 이계도함수 f''
term2_1 = -sin(sigma_t);
term2_2 = 2 * sin(2*sigma_t + sigma_p);
term2_3 = sin(sigma_p) .* cos(sigma_t + sigma_p);
f_double_prime = (term2_1 + term2_2 - term2_3) / denom;

%% 3. 최적점 (f' = 0 이고 f'' < 0 인 지점) 찾기
% f_prime이 양수(+)에서 음수(-)로 바뀌는 구간을 찾는다! (극대점 조건)
zero_idx = find(f_prime(1:end-1) > 0 & f_prime(2:end) <= 0);

if ~isempty(zero_idx)
    opt_idx = zero_idx(1); % 첫 번째 극대점
    opt_sigma_t_deg = sigma_t_deg(opt_idx);
else
    opt_sigma_t_deg = NaN; % 못 찾으면 예외 처리
end

%% 4. Plots (3x1 Subplot)
% 가로로 길고 세로로 꽉 차게 창 크기 설정
figure('Name', 'Function & Derivatives Analysis', 'Theme', 'light', 'Position', [100, 50, 800, 750]);

% -----------------------------------------------------------
% [Top] 원함수 f
% -----------------------------------------------------------
subplot(3, 1, 1);
plot(sigma_t_deg, f_val, 'b-', 'LineWidth', 2);
hold on; grid on;
if ~isnan(opt_sigma_t_deg)
    xline(opt_sigma_t_deg, 'r--', 'LineWidth', 1.5);
    plot(opt_sigma_t_deg, f_val(opt_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(opt_sigma_t_deg + 3, f_val(opt_idx), sprintf('Max at %.1f\\circ', opt_sigma_t_deg), 'Color', 'r', 'FontWeight', 'bold');
end
title(sprintf('원함수 f(\\sigma_p; \\sigma_t)  [\\sigma_p = %.1f\\circ]', sigma_p_deg), 'FontSize', 12, 'FontWeight', 'bold');
ylabel('f Value', 'FontWeight', 'bold');
xlim([0 180]);
hold off;

% -----------------------------------------------------------
% [Middle] 일계도함수 f'
% -----------------------------------------------------------
subplot(3, 1, 2);
plot(sigma_t_deg, f_prime, 'g-', 'LineWidth', 2);
hold on; grid on;
yline(0, 'k-', 'LineWidth', 1); % 기준선 y=0
if ~isnan(opt_sigma_t_deg)
    xline(opt_sigma_t_deg, 'r--', 'LineWidth', 1.5);
    plot(opt_sigma_t_deg, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
title('일계도함수 f'' (기울기)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('f'' Value', 'FontWeight', 'bold');
xlim([0 180]);
hold off;

% -----------------------------------------------------------
% [Bottom] 이계도함수 f''
% -----------------------------------------------------------
subplot(3, 1, 3);
plot(sigma_t_deg, f_double_prime, 'm-', 'LineWidth', 2);
hold on; grid on;
yline(0, 'k-', 'LineWidth', 1); % 기준선 y=0
if ~isnan(opt_sigma_t_deg)
    xline(opt_sigma_t_deg, 'r--', 'LineWidth', 1.5);
    plot(opt_sigma_t_deg, f_double_prime(opt_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
title('이계도함수 f'''' (볼록성)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('\sigma_t [degree]', 'FontWeight', 'bold');
ylabel('f'''' Value', 'FontWeight', 'bold');
xlim([0 180]);
hold off;