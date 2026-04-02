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
sigma_pc_deg = 30;
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
auto_save = true;
target_path = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260402 랩미팅 준비';

save_dir = fullfile(target_path);

if auto_save
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
        fprintf('\n>>> 아저씨가 전용 저장 폴더를 만들었어: %s\n', save_dir);
    end
end

%% 3. 격자(Meshgrid) 생성 
r_range = linspace(1, 1000, 2000); 
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

% 영역 그리기 (Safe / Unsafe 배경)
contourf(Px_grid, Py_grid, Acc_cmd_grid, [-100*acc_limit, -acc_limit, acc_limit, 100*acc_limit], 'LineColor', 'none');
colormap(ax, cmap); clim(ax, [-2*acc_limit, 2*acc_limit]); 

% 빨간색(+g), 보라색(-g) 경계선 (Max Acc Region) 그리기
contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
contour(Px_grid, Py_grid, Acc_cmd_grid, [-acc_limit -acc_limit], 'm', 'LineWidth', 1.5);

% DPG 도달 가능 영역 시각화 (동일 경로에 draw_DPG_Region.m이 있어야 해)
draw_DPG_Region(ax, sigma_pc_rad, r_f_max);

% =========================================================
% FRS + BRS 최종 Solution 구하기
% =========================================================
sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 500);

% 진짜 접점(Tangent Point) 좌표를 따로 확실하게 계산해두기!
r_contact = r_f_min * (cos(sigma_pc_rad))^2 / cos((sigma_t_contact + sigma_pc_rad)/2)^2;
plot_angle_contact = -pi/2 - sigma_t_contact;
x_contact = r_contact * cos(plot_angle_contact);
y_contact = r_contact * sin(plot_angle_contact);

if r_f_min > r_f_max
    % 겹치는 부분이 없을 때: 굵은 빨간색 글씨로 쾅!
    text(-17, 5, 'No Intersect', 'Color', 'r', 'FontSize', 28, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    
    valid_idx = r_min_traj <= max_r_plot_limit;
    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);
    x_min = r_min_traj(valid_idx) .* cos(plot_angle_traj);
    y_min = r_min_traj(valid_idx) .* sin(plot_angle_traj);
    
    plot(x_min, y_min, 'Color', [0 0.5 0], 'LineWidth', 2, 'LineStyle', '--');
    
    % 따로 계산해둔 접점 좌표에 빨간 별표 마커 콕 찍기!
    plot(x_contact, y_contact, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 1);
    
else
    % 겹치는 부분이 있을 때: 투명도 없는 찐 초록색으로 칠하기
    
    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    r_max_traj = r_f_max * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    
    % 발산 방지용 자르기
    valid_idx = (r_min_traj <= max_r_plot_limit) & (r_max_traj <= max_r_plot_limit);
    r_min_traj = r_min_traj(valid_idx);
    r_max_traj = r_max_traj(valid_idx);
    
    % [주의] 배열 크기 짝을 맞춰주기 위해 여기서도 valid_idx를 씌워야 해!
    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);
    
    % 좌표 변환
    x_min = r_min_traj .* cos(plot_angle_traj);
    y_min = r_min_traj .* sin(plot_angle_traj);
    x_max = r_max_traj .* cos(plot_angle_traj);
    y_max_curve = r_max_traj .* sin(plot_angle_traj);
    
    % 영역 채우기 (Solid Green)
    X_fill = [x_min, fliplr(x_max)];
    Y_fill = [y_min, fliplr(y_max_curve)];
    fill(X_fill, Y_fill, [0 0.8 0], 'EdgeColor', 'none', 'FaceAlpha', 1); 
    
    % r_f_min 궤적 테두리 그리기
    plot(x_min, y_min, 'g-', 'LineWidth', 2);
    
    % 따로 계산해둔 접점 좌표에 빨간 별표 마커 콕 찍기!
    plot(x_contact, y_contact, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 1);
    
    % 하방에 r_f_min < r_f < r_f_max 텍스트 표시
    txt_range = sprintf('%.2f m < r_f < %.2f m', r_f_min, r_f_max);
    text(-17, -12, txt_range, 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k', 'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
end

% =========================================================
% 랑데부 라인 및 교점 (별 마커) 표시 추가! (으헤~)
% =========================================================
plot_angle_intersect = -pi/2 - sigma_pc_rad;

% 1. 파란색 sigma_pc line 긋기
line_len = 30; % 넉넉하게 뻗어나가도록 설정
plot([0, line_len * cos(plot_angle_intersect)], ...
     [0, line_len * sin(plot_angle_intersect)], ...
     'b-', 'LineWidth', 1.5, 'DisplayName', '\sigma_t = \sigma_{pc} Line');

% 2. DPG (노란색 영역) 교점: r_f_max, 노란색 별
x_yellow = r_f_max * cos(plot_angle_intersect);
y_yellow = r_f_max * sin(plot_angle_intersect);
plot(x_yellow, y_yellow, 'p', 'MarkerSize', 16, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
text(x_yellow - 1, y_yellow - 2, sprintf('r_f = %.2f m', r_f_max), ...
     'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w');

% 3. BRS 직선 (녹색 영역 경계) 교점: r_f_min, 국방색 별
dark_green = [0.3 0.5 0.2]; % 국방색에 가까운 짙은 녹색
x_green = r_f_min * cos(plot_angle_intersect);
y_green = r_f_min * sin(plot_angle_intersect);
plot(x_green, y_green, 'p', 'MarkerSize', 16, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', dark_green);
text(x_green - 1, y_green - 2.5, sprintf('r_{f, min} = %.2f m', r_f_min), ...
     'Color', dark_green, 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w');

% 제목 업데이트
title(sprintf('BRS Region Analysis: \\sigma_{pc} = %.1f^\\circ', sigma_pc_deg), 'FontSize', 14, 'FontWeight', 'bold');

% Target 화살표
quiver(0, 0, 0, 15, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% 축 범위 및 라벨
xlim([-30, 2]); ylim([-16, 16]);
xlabel('x (m)', 'FontSize', 12); 
ylabel('y (m)', 'FontSize', 12);

% 범례(Legend) 달기 
h_red   = plot(nan, nan, 'r', 'LineWidth', 1.5);
h_purp  = plot(nan, nan, 'm', 'LineWidth', 1.5);
h_safe  = patch(nan, nan, [0.7 0.8 1.0], 'EdgeColor', 'none'); 
h_reach = patch(nan, nan, [0.8 0.7 0], 'EdgeColor', 'none'); 
h_inter = patch(nan, nan, [0 0.8 0], 'EdgeColor', 'none'); 
h_tangent = plot(nan, nan, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_sigma_line = plot(nan, nan, 'b-', 'LineWidth', 1.5);
h_star_y = plot(nan, nan, 'p', 'MarkerSize', 12, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
h_star_g = plot(nan, nan, 'p', 'MarkerSize', 12, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', dark_green);

legend([h_red, h_purp, h_safe, h_reach, h_inter, h_tangent, h_sigma_line, h_star_y, h_star_g], ...
    {'+g Limit (Unsafe)', '-g Limit (Unsafe)', 'Safe Region', 'DPG Reachable Region', ...
     'Intersect Region', 'Tangent Point', '\sigma_{pc} Line', 'r_{f, DPG} Intersect', 'r_{f, BRS} Intersect'}, ...
    'Location', 'northwest', 'FontSize', 10);

% =========================================================
% 이미지로 굽기!
% =========================================================
if auto_save
    filename = sprintf('Intersection_Result_%.1f_deg.png', sigma_pc_deg);
    full_path = fullfile(save_dir, filename);
    exportgraphics(ax, full_path, 'Resolution', 300); 
    fprintf('    [저장 완료] %s\n', filename);
    fprintf('>>> 저장 끝! 선생, 고생했어~ 으헤.\n');
end