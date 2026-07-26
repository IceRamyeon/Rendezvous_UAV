clear; clc; close all;

%% 1. 기본 파라미터 설정 (각도 리스트를 먼저 정의해야 해~)
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle) - 이 값들에 따라 6개 플롯 생성
sigma_pc_deg_list = [62 62.3 62.5 62.6 62.8 63];  
sigma_pc_rad_list = deg2rad(sigma_pc_deg_list);

% Sampling Parameters
sampling_freq = 1; % Sampling하는 점들의 간격 [m]

% BRS Parameters
r_f = 2.0;

% =========================================================
% [추가된 파라미터] 궤적 발산 방지용 최대 그리기 반경
% =========================================================
% Case 6처럼 수학적으로 발산하는 궤적을 싹둑 잘라버리기 위한 제한 거리야.
% 축 범위가 대략 [-50, 30]이니까 100m 정도면 충분해~ 으헤.
max_r_plot_limit = 1000; 

%% 2. 자동 저장 설정 (선생이 원하는 맞춤형 경로!)
% 주의: 선생 PC에 있는 OneDrive 경로로 꼭 수정해서 돌려야 해!
auto_save = true;
target_path = 'C:\Users\최혁재\Desktop\대학자료\AISL 연구실\발표 사진 및 영상\260326 랩미팅 준비';

% 각도 6개를 폴더 이름에 쏙쏙 집어넣기
folder_name = sprintf('BRS 분석 [%.1f %.1f %.1f %.1f %.1f %.1f]', sigma_pc_deg_list);

% 경로랑 폴더 이름 합치기
save_dir = fullfile(target_path, folder_name);

% 저장 폴더가 없으면 미리 만들어두기
if auto_save
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
        fprintf('\n>>> 아저씨가 전용 저장 폴더를 만들었어: %s\n', save_dir);
    else
        fprintf('\n>>> 오, 폴더가 이미 있네? 바로 저장할게: %s\n', save_dir);
    end
end

%% 3. 격자(Meshgrid) 생성 (공통)
% R: 1m ~ 200m, Theta: 90도 ~ 270도
r_range = linspace(1, 200, 500); 
theta_range = linspace(deg2rad(90), deg2rad(270), 400);
[R_grid, Theta_grid] = meshgrid(r_range, theta_range);

% 좌표 변환
Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);

% 기하학적 각도 계산
Lam_grid = Theta_grid + pi;       % 시선각
Sigma_t_grid = psi_t - Lam_grid;  % 타겟 리드각

%% 4. Custom Colormap 생성
cmap = zeros(256, 3);
cmap(1:50, :)   = repmat([0.8 0.6 1.0], 50, 1);     % Purple (-g 영역)
cmap(51:180, :) = repmat([0.7 0.8 1.0], 130, 1);    % Blue (Safe 영역)
cmap(181:256, :) = repmat([1.0 0.7 0.7], 76, 1);    % Red (+g 영역)

%% 5. 샘플링된 점들을 저장할 셀 배열 준비
sampled_red_points = cell(1, 6); 

%% 6. 6개 서브플롯 생성 및 개별 저장 루프
figure('Name', 'Saturation Map with Trajectories', 'Theme', 'light', 'Position', [50, 50, 1100, 850]);

for i = 1:6
    current_sigma_pc = sigma_pc_rad_list(i);
    current_sigma_p = current_sigma_pc; 
    current_sigma_ref = current_sigma_pc; % 궤적용 sigma_ref
    
    % --- 가속도 계산 ---
    Term1 = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(current_sigma_p));
    Term2 = V * k_gain * (current_sigma_p - current_sigma_pc);
    Acc_cmd_grid = Term1 - Term2;
    
    % 서브플롯 배치
    ax = subplot(2, 3, i); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    % 영역 그리기
    contourf(Px_grid, Py_grid, Acc_cmd_grid, [-100*acc_limit, -acc_limit, acc_limit, 100*acc_limit], 'LineColor', 'none');
    colormap(gca, cmap); clim(gca, [-2*acc_limit, 2*acc_limit]); 
    
    % --- 빨간색 경계선(+g) 그리고 길이 계산 & 샘플링 ---
    [C_red, ~] = contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    
    idx = 1;
    temp_sampled_x = [];
    temp_sampled_y = [];
    
    while idx < size(C_red, 2)
        num_points = C_red(2, idx); 
        x_pts = C_red(1, idx+1 : idx+num_points);
        y_pts = C_red(2, idx+1 : idx+num_points);
        
        distances = sqrt(diff(x_pts).^2 + diff(y_pts).^2);
        cum_dist = [0, cumsum(distances)];
        
        if cum_dist(end) > 0
            dq = 0:sampling_freq:cum_dist(end); 
            [unique_cum_dist, unique_idx] = unique(cum_dist);
            
            if length(unique_cum_dist) > 1
                sq_x = interp1(unique_cum_dist, x_pts(unique_idx), dq);
                sq_y = interp1(unique_cum_dist, y_pts(unique_idx), dq);
                temp_sampled_x = [temp_sampled_x, sq_x];
                temp_sampled_y = [temp_sampled_y, sq_y];
            end
        end
        idx = idx + num_points + 1; 
    end
    
    % 보라색 경계선(-g) 그리기
    contour(Px_grid, Py_grid, Acc_cmd_grid, [-acc_limit -acc_limit], 'm', 'LineWidth', 1.5);
    sampled_red_points{i} = [temp_sampled_x', temp_sampled_y'];
    
    % --- 이론적 Reachable Region (노란색 영역) ---
    draw_DPG_Region(ax, current_sigma_pc, r_f);
    
    % --- 샘플링된 점들로부터의 Trajectory 그리기 ---
    for k = 1:length(temp_sampled_x)
        x_s = temp_sampled_x(k);
        y_s = temp_sampled_y(k);
        
        % 우반 평면(x > 0)에 있는 점들은 궤적 그리기에서 스킵!
        if x_s > 0
            continue;
        end
        
        r_s = sqrt(x_s^2 + y_s^2);
        
        % 방위각 역산을 통한 초기 sigma_t 구하기
        plot_angle_s = atan2(y_s, x_s);
        if plot_angle_s < 0
            plot_angle_s = plot_angle_s + 2*pi;
        end
            
        sigma_ts = -pi/2 - plot_angle_s;
        
        % BRS 수식을 이용해 이 궤적만의 고유한 상수 K 도출
        K = r_s * cos((sigma_ts + current_sigma_ref)/2)^2;
        
        % DPG 궤적은 최종적으로 -sigma_ref를 향해 수렴함
        sigma_t_traj = linspace(sigma_ts, -current_sigma_ref, 100);
        r_traj_full = K ./ cos((sigma_t_traj + current_sigma_ref)/2).^2;
        
        % -----------------------------------------------------------------
        % =========================================================
        % [수정된 부분] 궤적 클리핑(Clipping) 로직 강화
        % =========================================================
        % 1. 기존: 타겟 반경(r_f) 안으로 들어가는 경우만 체크 (valid_idx = r_traj >= r_f;)
        % 2. 수정: 발산 방지를 위해 너무 멀어지는 경우(max_r_plot_limit)도 함께 체크!!
        
        valid_idx = (r_traj_full >= r_f) & (r_traj_full <= max_r_plot_limit);
        
        sigma_t_traj = sigma_t_traj(valid_idx);
        r_traj = r_traj_full(valid_idx);
        % -----------------------------------------------------------------
        
        if ~isempty(sigma_t_traj)
            plot_angle_traj = -pi/2 - sigma_t_traj;
            x_traj = r_traj .* cos(plot_angle_traj);
            y_traj = r_traj .* sin(plot_angle_traj);
            
            % 궤적 그리기 (초록색 반투명 선)
            plot(x_traj, y_traj, 'Color', [0.1 0.7 0.1 0.6], 'LineWidth', 1.0);
        end
    end
    
    % 샘플링된 시작점에 초록색 동그라미 콕콕 찍기
    if ~isempty(temp_sampled_x)
        scatter(temp_sampled_x, temp_sampled_y, 15, 'g', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % 제목 업데이트
    title(sprintf('Case %d: \\sigma_{pc} = %.1f^\\circ\nSampled Pts: %d', ...
        i, sigma_pc_deg_list(i), length(temp_sampled_x)), 'FontSize', 10);
    
    % Target 화살표
    quiver(0, 0, 0, 15, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    % 축 범위 (여기가 이제 예쁘게 꽉 찰 거야~ 으헤.)
    xlim([-37, 3]); ylim([-15, 25]);
    if i > 3, xlabel('x (m)'); end
    if mod(i, 3) == 1, ylabel('y (m)'); end
    
    % =========================================================
    % 각각의 서브플롯에 범례(Legend) 달기
    % =========================================================
    % 꼬이지 않게 투명한 더미(Dummy) 객체를 만들어서 깔끔하게 표시할게~
    h_red   = plot(nan, nan, 'r', 'LineWidth', 1.5);
    h_purp  = plot(nan, nan, 'm', 'LineWidth', 1.5);
    h_safe  = patch(nan, nan, [0.7 0.8 1.0], 'EdgeColor', 'none'); 
    h_traj  = plot(nan, nan, 'Color', [0.1 0.7 0.1], 'LineWidth', 1.5);
    h_reach = patch(nan, nan, [0.8 0.7 0], 'EdgeColor', 'none');
    
    legend([h_red, h_purp, h_safe, h_traj, h_reach], ...
        {'+g Limit (Unsafe)', '-g Limit (Unsafe)', 'Safe Region', 'Trajectory', 'Reachable Region'}, ...
        'Location', 'southwest', 'FontSize', 8);
    
    % =========================================================
    % 현재 완성된 서브플롯(ax)을 바로 개별 이미지로 굽기!
    % =========================================================
    if auto_save
        filename = sprintf('Subplot_%02d_Result.png', i);
        full_path = fullfile(save_dir, filename);
        exportgraphics(ax, full_path, 'Resolution', 150); 
        fprintf('    [저장 완료] %s\n', filename);
    end
end

% 전체 창 제목
sgtitle('BRS Dangerous Regions with DPG Trajectories', 'FontSize', 14, 'FontWeight', 'bold');

if auto_save
    fprintf('>>> 저장 끝! 선생, 고생했어~ 으헤.\n');
end