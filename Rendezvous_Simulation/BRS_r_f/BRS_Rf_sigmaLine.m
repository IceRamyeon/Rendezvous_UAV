clear; clc; close all;

%% 0. 디렉토리 추가
addpath('./Rendezvous_Simulation/Plotting_function');
addpath('./Rendezvous_Simulation/BRS&max_acc');

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle)
sigma_pc_deg = 65;
sigma_pc_rad = deg2rad(sigma_pc_deg);

% Rendezvous에서 r_f > r_f_max
r_f_max = 2.0; % 기존의 r_f를 r_f_max로 설정

% 업로드된 파일의 수식을 이용해 r_f_min 도출
epsilon = 1e-7;
sigma_t_calc = linspace(0, pi, 50000);
y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);

[y_max, max_idx] = max(y_calc);

% 여기가 핵심! Max Acc Region과 정확히 접하는 각도야~
sigma_t_contact = sigma_t_calc(max_idx); 

% 구한 최대 랑데부 반경을 최소 요구 반경(r_f_min)으로 설정
r_f_min = (y_max * V^2) / (2 * acc_limit);

max_r_plot_limit = 1000; % 궤적 발산 방지용

%% 2. 자동 저장 설정
auto_save = false;
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260402 랩미팅 준비';

save_dir = fullfile(target_path);

if auto_save
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
        fprintf('\n>>> 전용 저장 폴더: %s\n', save_dir);
    end
end

%% 3. 격자(Meshgrid) 생성 
r_target_radius = 2.0; % 기준 반경 설정

r_range = [linspace(r_target_radius, 50, 1500), linspace(50.5, 1000, 500)]; 
theta_range = linspace(deg2rad(90), deg2rad(270), 400);
[R_grid, Theta_grid] = meshgrid(r_range, theta_range);

Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);

Lam_grid = Theta_grid + pi;       
Sigma_t_grid = psi_t - Lam_grid;

%% 4. Custom Colormap 생성
cmap = zeros(256, 3);
cmap(1:50, :)   = repmat([0.8 0.6 1.0], 50, 1);     % Purple (-g 영역)
cmap(51:180, :) = repmat([0.7 0.8 1.0], 130, 1);    % Blue (Safe 영역)
cmap(181:256, :) = repmat([1.0 0.7 0.7], 76, 1);    % Red (+g 영역)

%% 5. 단일 플롯 생성 및 그리기
figure('Name', 'Guidance Reachable Region Analysis', 'Theme', 'light', 'Position', [100, 100, 800, 700]);
ax = gca; 
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

% --- 가속도 계산 ---
Term1 = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(sigma_pc_rad));
Term2 = V * k_gain * (sigma_pc_rad - sigma_pc_rad); 
Acc_cmd_grid = Term1 - Term2;

% 혹시 모를 예외 방지로 2m 미만 영역은 NaN 처리
Acc_cmd_grid(R_grid < r_target_radius) = NaN;

% 영역 그리기 (Safe / Unsafe 배경)
contourf(Px_grid, Py_grid, Acc_cmd_grid, [-100*acc_limit, -acc_limit, acc_limit, 100*acc_limit], 'LineColor', 'none');
colormap(ax, cmap); clim(ax, [-2*acc_limit, 2*acc_limit]); 

% 빨간색(+g), 보라색(-g) 경계선 (Max Acc Region) 그리기
contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
contour(Px_grid, Py_grid, Acc_cmd_grid, [-acc_limit -acc_limit], 'm', 'LineWidth', 1.5);

% DPG 도달 가능 영역 시각화
draw_DPG_Region(ax, sigma_pc_rad, r_f_max);

% =========================================================
% FRS + BRS 최종 Solution 구하기
% =========================================================
sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 500);

% 접점(Tangent Point) 좌표 및 원점으로부터의 거리 계산
r_contact = r_f_min * (cos(sigma_pc_rad))^2 / cos((sigma_t_contact + sigma_pc_rad)/2)^2;
plot_angle_contact = -pi/2 - sigma_t_contact;
x_contact = r_contact * cos(plot_angle_contact);
y_contact = r_contact * sin(plot_angle_contact);

if r_f_min > r_f_max
    text(-17, 5, 'No Intersect', 'Color', 'r', 'FontSize', 28, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    
    % [수정] 2m 원 주위에 선이 남지 않도록, 애초에 2m 이상인 데이터만 살리기
    valid_idx = (r_min_traj >= r_target_radius) & (r_min_traj <= max_r_plot_limit);
    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);
    x_min = r_min_traj(valid_idx) .* cos(plot_angle_traj);
    y_min = r_min_traj(valid_idx) .* sin(plot_angle_traj);
    
    plot(x_min, y_min, 'Color', [0 0.5 0], 'LineWidth', 2, 'LineStyle', '--');
    
    % Tangent Point가 반경 2m 이상일 때만 빨간 별표 마커 생성
    if r_contact >= r_target_radius
        plot(x_contact, y_contact, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 1);
    end
    
else
    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    r_max_traj = r_f_max * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    
    % [수정] 원 주위를 감싸는 잔상을 제거하기 위해 r_min_traj가 2m 이상인 구간만 유효화!
    valid_idx = (r_min_traj >= r_target_radius) & (r_max_traj <= max_r_plot_limit);
    r_min_traj = r_min_traj(valid_idx);
    r_max_traj = r_max_traj(valid_idx);
    
    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);
    
    x_min = r_min_traj .* cos(plot_angle_traj);
    y_min = r_min_traj .* sin(plot_angle_traj);
    x_max = r_max_traj .* cos(plot_angle_traj);
    y_max_curve = r_max_traj .* sin(plot_angle_traj);
    
    X_fill = [x_min, fliplr(x_max)];
    Y_fill = [y_min, fliplr(y_max_curve)];
    fill(X_fill, Y_fill, [0 0.8 0], 'EdgeColor', 'none', 'FaceAlpha', 1); 
    
    plot(x_min, y_min, 'g-', 'LineWidth', 2);
    
    % Tangent Point가 반경 2m 이상일 때만 빨간 별표 마커 생성
    if r_contact >= r_target_radius
        plot(x_contact, y_contact, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 1);
    end
    
    txt_range = sprintf('%.2f m < r_f < %.2f m', r_f_min, r_f_max);
    text(-17, -12, txt_range, 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k', 'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
end

% =========================================================
% 랑데부 라인 표시
% =========================================================
plot_angle_intersect = -pi/2 - sigma_pc_rad;

% 파란색 sigma_pc line (반경 2m 지점부터 시작하도록 유지)
line_len = 30; 
plot([r_target_radius * cos(plot_angle_intersect), line_len * cos(plot_angle_intersect)], ...
     [r_target_radius * sin(plot_angle_intersect), line_len * sin(plot_angle_intersect)], ...
     'b-', 'LineWidth', 1.5, 'DisplayName', '\sigma_t = \sigma_{pc} Line');

% [삭제 완료] r_f DPG 및 r_f BRS의 별 모양 마커 표시 코드 완전 제거

% 제목 업데이트
title(sprintf('BRS Region Analysis: \\sigma_{pc} = %.1f^\\circ', sigma_pc_deg), 'FontSize', 14, 'FontWeight', 'bold');

% [수정] 중심에 Target을 빨간색 삼각형으로 생성 (높이 2m, y축 범위 [-1, +1], 위를 보는 방향)
target_x = [0, -1, 1];
target_y = [1, -1, -1];
fill(ax, target_x, target_y, 'r', 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% 축 범위 및 라벨
xlim([-700, 100]); ylim([-400, 400]);
xlabel('x (m)', 'FontSize', 12); 
ylabel('y (m)', 'FontSize', 12);

% 범례(Legend) 설정 (불필요한 항목 정리)
h_red   = plot(nan, nan, 'r', 'LineWidth', 1.5);
h_purp  = plot(nan, nan, 'm', 'LineWidth', 1.5);
h_safe  = patch(nan, nan, [0.7 0.8 1.0], 'EdgeColor', 'none'); 
h_reach = patch(nan, nan, [0.8 0.7 0], 'EdgeColor', 'none'); 
h_inter = patch(nan, nan, [0 0.8 0], 'EdgeColor', 'none'); 
h_tangent = plot(nan, nan, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_sigma_line = plot(nan, nan, 'b-', 'LineWidth', 1.5);

legend([h_red, h_purp, h_safe, h_reach, h_inter, h_tangent, h_sigma_line], ...
    {'+g Limit (Unsafe)', '-g Limit (Unsafe)', 'Safe Region', 'DPG Reachable Region', ...
     'Intersect Region', 'Tangent Point', '\sigma_{pc} Line'}, ...
    'Location', 'northwest', 'FontSize', 10);

%% 6. 자동 저장
if auto_save
    filename = sprintf('Intersection_Result_%.1f_deg.png', sigma_pc_deg);
    full_path = fullfile(save_dir, filename);
    exportgraphics(ax, full_path, 'Resolution', 300); 
    fprintf('    [저장 완료] %s\n', filename);
    fprintf('>>> 저장 끝! 선생, 고생했어~ 으헤.\n');
end