% DPG Final Points 집합과 특정 궤적 간의 최단 거리 찾기
clc; clear; close all;

%% 1. DPG Final Points (Test Point Set) 계산
Vp = 20;           %[cite: 7]
Vt = 20;           %[cite: 7]
gamma_t_deg = 90;       %[cite: 7]
gamma_t = deg2rad(gamma_t_deg); %[cite: 7]
T = 3;             %[cite: 7]

% Pursuer의 초기 위치 (Target 기준 프레임)
x0 = -800; %[cite: 7]
y0 = 500; %[cite: 7]

sigma_pc_deg_set = -180:10:180; %[cite: 7]
final_points = zeros(length(sigma_pc_deg_set), 2); %[cite: 7]

for i = 1:length(sigma_pc_deg_set)
    sigma_pc_val = deg2rad(sigma_pc_deg_set(i)); %[cite: 7]
    
    % 상대 운동 방정식[cite: 7]
    dydt = @(t, Y) [
        Vp * cos(atan2(-Y(2), -Y(1)) + sigma_pc_val) - Vt * cos(gamma_t);
        Vp * sin(atan2(-Y(2), -Y(1)) + sigma_pc_val) - Vt * sin(gamma_t)
    ];
    
    [~, Y] = ode45(dydt, [0 T], [x0; y0]); %[cite: 7]
    final_points(i, :) = Y(end, :); %[cite: 7]
end

%% 2. 특정 궤적 (sigma_pc = 62.6도) 계산
r_f = 2.0;         %[cite: 7]
r_target_radius = 2.0;     %[cite: 7]
max_r_plot_limit = 20000;  %[cite: 7]

sigma_pc_deg_traj = 62.6; %[cite: 7]
sigma_pc_rad_traj = deg2rad(sigma_pc_deg_traj); %[cite: 7]

% 해석적 궤적 계산[cite: 7]
sigma_t_traj = linspace(-sigma_pc_rad_traj, pi - sigma_pc_rad_traj - 1e-5, 20000); %[cite: 7]
denominator = cos((sigma_t_traj + sigma_pc_rad_traj) / 2).^2; %[cite: 7]
r_traj = r_f * cos(sigma_pc_rad_traj)^2 ./ denominator; %[cite: 7]
r_traj = min(r_traj, max_r_plot_limit); %[cite: 7]

valid_idx = isfinite(r_traj) & (r_traj >= r_target_radius); %[cite: 7]
plot_sigma_t = sigma_t_traj(valid_idx); %[cite: 7]
plot_r = r_traj(valid_idx); %[cite: 7]

x_traj = []; %[cite: 7]
y_traj = []; %[cite: 7]
if ~isempty(plot_r) %[cite: 7]
    plot_angle = -pi/2 - plot_sigma_t; %[cite: 7]
    x_traj = plot_r .* cos(plot_angle); %[cite: 7]
    y_traj = plot_r .* sin(plot_angle); %[cite: 7]
end

%% 3. Boundary 통과 여부 판별 및 최적의 sigma_pc 탐색
in = inpolygon(final_points(:,1), final_points(:,2), x_traj, y_traj);

if any(in) && any(~in)
    % Boundary가 FRS 집합을 통과하는 경우
    % 내부에 있는 점들 중 sigma_pc_deg_traj(62.6도)와 가장 차이가 적은 점 탐색
    valid_in_idx = find(in);
    [~, min_diff_idx] = min(abs(sigma_pc_deg_set(valid_in_idx) - sigma_pc_deg_traj));
    best_fp_idx = valid_in_idx(min_diff_idx);
    mode_flag = 'CROSS';
    
    % 해당 점과 궤적 사이의 최단 거리 계산 (출력용 및 그래프용)
    if ~isempty(x_traj)
        distances = sqrt((x_traj - final_points(best_fp_idx, 1)).^2 + (y_traj - final_points(best_fp_idx, 2)).^2);
        [min_dist, best_traj_idx] = min(distances);
    else
        min_dist = inf;
        best_traj_idx = 1;
    end
else
    % 완전히 밖에 있거나 안에 있는 경우: 궤적과 제일 가까운 점 탐색
    min_dist = inf; %[cite: 7]
    best_fp_idx = 1; %[cite: 7]
    best_traj_idx = 1; %[cite: 7]
    mode_flag = 'SHORTEST';

    if ~isempty(x_traj) %[cite: 7]
        for i = 1:size(final_points, 1) %[cite: 7]
            % 하나의 Final Point와 궤적 상의 모든 점 사이의 거리 계산[cite: 7]
            distances = sqrt((x_traj - final_points(i, 1)).^2 + (y_traj - final_points(i, 2)).^2); %[cite: 7]
            
            [local_min_dist, local_min_idx] = min(distances); %[cite: 7]
            
            % 전체 최솟값 업데이트[cite: 7]
            if local_min_dist < min_dist %[cite: 7]
                min_dist = local_min_dist; %[cite: 7]
                best_fp_idx = i; %[cite: 7]
                best_traj_idx = local_min_idx; %[cite: 7]
            end
        end
    end
end

%% 4. 결과 시각화 및 터미널 출력
figure('Name', 'Shortest Point to Final Points', 'Theme', 'light', 'Position', [100, 100, 1000, 750]); %[cite: 7]
hold on; grid on; axis equal; %[cite: 7]

% 1) 궤적 그리기 (Dark Green)[cite: 7]
dark_green = [0.0, 0.6, 0.0]; %[cite: 7]
if ~isempty(x_traj) %[cite: 7]
    plot(x_traj, y_traj, 'Color', dark_green, 'LineWidth', 2, 'DisplayName', sprintf('\\sigma_{pc} = %.1f^\\circ Trajectory', sigma_pc_deg_traj)); %[cite: 7]
end

% 2) Final Points 집합 그리기 (Red line & markers)[cite: 7]
plot(final_points(:,1), final_points(:,2), 'r-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Points Set'); %[cite: 7]

% 3) 가장 가까운 점들 마킹 및 선 연결[cite: 7]
if ~isempty(x_traj) %[cite: 7]
    closest_fp = final_points(best_fp_idx, :); %[cite: 7]
    closest_traj_pt = [x_traj(best_traj_idx), y_traj(best_traj_idx)]; %[cite: 7]
    
    % 궤적 상의 최단 거리 점 (Purple Star)[cite: 7]
    purple = [0.5, 0.0, 0.5]; %[cite: 7]
    plot(closest_traj_pt(1), closest_traj_pt(2), 'p', 'MarkerSize', 15, 'MarkerFaceColor', purple, 'MarkerEdgeColor', 'k', 'DisplayName', 'Closest Traj Point'); %[cite: 7]

    % Final Point 중 선택된 점 (Blue Circle)
    plot(closest_fp(1), closest_fp(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Selected Final Point'); %[cite: 7]
    
    % 두 점을 잇는 점선[cite: 7]
    plot([closest_fp(1), closest_traj_pt(1)], [closest_fp(2), closest_traj_pt(2)], 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off'); %[cite: 7]
    
    % --- Command Window 출력 부분 ---
    fprintf('\n========================================\n');
    fprintf('             계산 결과             \n');
    fprintf('========================================\n');
    if strcmp(mode_flag, 'CROSS')
        fprintf('▶ 판별 상태: Boundary가 FRS 집합을 통과했어!\n');
        fprintf('▶ 선택 기준: Boundary 내부 점 중 sigma_pc_deg_traj(%.1f도)와 가장 차이가 적은 점\n', sigma_pc_deg_traj);
    else
        fprintf('▶ 판별 상태: Boundary가 FRS 집합을 가로지르지 않았어.\n');
        fprintf('▶ 선택 기준: Boundary와 가장 짧은 거리를 가지는 점\n');
    end
    fprintf('▶ 궤적과의 최단 거리: %.3f m\n', min_dist);
    fprintf('▶ 선택된 Final Point의 sigma_pc: %d도\n', sigma_pc_deg_set(best_fp_idx));
    fprintf('========================================\n\n');
end

% 4) Target 표시[cite: 7]
target_x = [0, -1, 0, 1]; %[cite: 7]
target_y = [1, -1, -0.3, -1]; %[cite: 7]
fill(target_x, target_y, 'k', 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Target'); %[cite: 7]
theta_circle = linspace(0, 2*pi, 300); %[cite: 7]
plot(r_target_radius * cos(theta_circle), r_target_radius * sin(theta_circle), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); %[cite: 7]

xlabel('x (m)', 'FontSize', 12); %[cite: 7]
ylabel('y (m)', 'FontSize', 12); %[cite: 7]
title(sprintf('Minimum Distance between Final Points and Trajectory (\\sigma_{pc} = %.1f^\\circ)', sigma_pc_deg_traj), 'FontSize', 15, 'FontWeight', 'bold'); %[cite: 7]
legend('show', 'Location', 'bestoutside'); %[cite: 7]

xlim([-1000 0]); %[cite: 7]
ylim([-100 900]); %[cite: 7]