clear; clc; close all;

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
r_f_max = 2.0;        % 최대 랑데부 반경

% 선생이 요청한 각도 리스트
sigma_p0_list = [65, 75, 85];
epsilon = 1e-7;
max_r_plot_limit = 1000;
r_target_radius = 2.0;

%% 2. 플롯 생성
figure('Name', 'Intersection Region Analysis', 'Theme', 'light', 'Position', [100, 100, 800, 700]);
ax = gca;
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

%% 3. 각도별 영역 그리기 (텍스트 부분 제외)
for i = 1:length(sigma_p0_list)
    sigma_pc_deg = sigma_p0_list(i);
    sigma_pc_rad = deg2rad(sigma_pc_deg);

    % r_f_min 도출
    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
    [y_max, max_idx] = max(y_calc);
    r_f_min = (y_max * V^2) / (2 * acc_limit);

    % FRS + BRS 궤적 계산
    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 1000);

    r_min_traj = r_f_min * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;
    r_max_traj = r_f_max * (cos(sigma_pc_rad))^2 ./ cos((sigma_t_traj + sigma_pc_rad)/2).^2;

    % 유효 구간 설정 (발산 및 잔상 방지)
    valid_idx = (r_min_traj >= r_target_radius) & (r_max_traj <= max_r_plot_limit);
    r_min_traj = r_min_traj(valid_idx);
    r_max_traj = r_max_traj(valid_idx);

    plot_angle_traj = -pi/2 - sigma_t_traj(valid_idx);

    x_min = r_min_traj .* cos(plot_angle_traj);
    y_min = r_min_traj .* sin(plot_angle_traj);
    x_max = r_max_traj .* cos(plot_angle_traj);
    y_max_curve = r_max_traj .* sin(plot_angle_traj);

    % 초록색 Intersection Region 채우기 (가시성을 위해 투명도 적용)
    X_fill = [x_min, fliplr(x_max)];
    Y_fill = [y_min, fliplr(y_max_curve)];
    fill(X_fill, Y_fill, [0 0.8 0], 'EdgeColor', 'k', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
end

%% 4. Target 및 부가 요소 표시
% Target 삼각형 생성
target_x = [0, -1, 1];
target_y = [1, -1, -1];
fill(ax, target_x, target_y, 'r', 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Target');

% Target Velocity 화살표 추가
quiver(0, 0, 0, 20, 'Color', [0.8 0.2 0], 'LineWidth', 2, 'MaxHeadSize', 1.5, 'HandleVisibility', 'off');
text(-40, 30, 'Target Velocity', 'FontSize', 11, 'FontWeight', 'bold');

% 축 범위 및 라벨
xlim([-250, 10]); ylim([-60, 200]);
xlabel('x (m)', 'FontSize', 12);
ylabel('y (m)', 'FontSize', 12);

%% 5. 수동 텍스트 및 화살표 위치 조정
% 화살표를 반대로 하고 싶으면 \leftarrow 대신 \rightarrow 를 쓰면 돼.

% [1] sigma_p0 = 65도
x_65 = -125; 
y_65 = 60;
text(x_65, y_65, '\sigma_{p0} = 65^\circ', 'FontSize', 12, 'FontWeight', 'bold');
quiver(x_65, y_65, -10, -7, 0, 'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 2);

% [2] sigma_p0 = 75도
x_75 = -230; 
y_75 =  30;
text(x_75, y_75, '\sigma_{p0} = 75^\circ', 'FontSize', 12, 'FontWeight', 'bold');
quiver(x_75 + 35, y_75, 10, 7, 0, 'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 2);

% [3] sigma_p0 = 85도
x_85 = -170; 
y_85 = -10;
text(x_85, y_85, '\sigma_{p0} = 85^\circ', 'FontSize', 12, 'FontWeight', 'bold');
quiver(x_85 + 10, y_85 + 5, 7, 10, 0, 'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 2);

fprintf('>>> 플롯 생성 완료\n');