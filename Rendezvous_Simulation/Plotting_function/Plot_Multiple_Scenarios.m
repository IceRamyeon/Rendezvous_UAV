%% Plot_Multiple_Scenarios.m
clc; clear; close all;

% -----------------------------------------------------------
% 1. 플롯할 파일 이름 배열 설정 (여기에 파일명 추가/삭제)
% -----------------------------------------------------------
file_list = {
    'Scenario_67.mat', ...
    'Scenario_70.mat', ...
    'Scenario_73.mat'
};

% 파일이 저장된 폴더 경로
data_dir = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260703 미팅 준비\Sim4.3';
% 자동으로 PNG를 저장할 폴더 경로
save_dir = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260703 미팅 준비\Sim4.3';

% -----------------------------------------------------------
% 2. 플롯 및 색상 초기화
% -----------------------------------------------------------
colors = lines(length(file_list)); 

% 창 분할: 궤적용 정사각형 창 1개 + 데이터용 길쭉한 창 2개
fig_traj = figure('Name', 'Trajectory Analysis', 'Position', [100, 100, 800, 800], 'Theme', 'light'); 
fig1     = figure('Name', 'Rendezvous Simulation Results (1/2)', 'Position', [150, 150, 1200, 900], 'Theme', 'light');
fig2     = figure('Name', 'Rendezvous Simulation Results (2/2)', 'Position', [200, 200, 1200, 900], 'Theme', 'light');

% 범례 핸들 배열
h_traj   = zeros(1, length(file_list));
h_lines1 = zeros(1, length(file_list));
h_lines2 = zeros(1, length(file_list));
legend_labels = cell(1, length(file_list));

% -----------------------------------------------------------
% 3. 반복문을 통한 데이터 로드 및 플로팅
% -----------------------------------------------------------
for i = 1:length(file_list)
    full_path = fullfile(data_dir, file_list{i});
    
    if ~exist(full_path, 'file')
        warning('으헤... %s 파일이 지정된 경로에 없어. 건너뛸게.', file_list{i});
        continue;
    end
    
    loaded = load(full_path);
    data = loaded.log_data;
    t = data.Time;
    
    [~, name, ~] = fileparts(file_list{i});
    legend_labels{i} = strrep(name, '_', '\_');
    
    % ==========================================
    % 0. Trajectory 독립 창 (정사각형)
    % ==========================================
    figure(fig_traj);
    ax0 = gca; hold on; grid on; % ax0에 핸들 저장
    plot(data.Trajectory.Xt, data.Trajectory.Yt, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off'); 
    h_traj(i) = plot(data.Trajectory.Xp, data.Trajectory.Yp, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('0. Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal;
    
    % ==========================================
    % 첫 번째 데이터 창 (1 ~ 3번 데이터, 3행 1열)
    % ==========================================
    figure(fig1);
    
    % --- 1. Relative Distance ---
    ax1 = subplot(3, 1, 1); hold on; grid on; % ax1에 핸들 저장
    h_lines1(i) = plot(t, data.Relative_Distance, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('1. Relative Distance'); xlabel('Time (s)'); ylabel('Range (m)');
    
    % --- 2. Closing Velocity ---
    ax2 = subplot(3, 1, 2); hold on; grid on; % ax2에 핸들 저장
    plot(t, data.Closing_Velocity, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('2. Closing Velocity'); xlabel('Time (s)'); ylabel('V_c (m/s)');
    
    % --- 3. Pursuer Acc ---
    ax3 = subplot(3, 1, 3); hold on; grid on; % ax3에 핸들 저장
    plot(t, data.Pursuer_Acc, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('3. Pursuer Acc'); xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
    
    % ==========================================
    % 두 번째 데이터 창 (4 ~ 7번 데이터, 4행 1열)
    % ==========================================
    figure(fig2);
    
    % --- 4. Acc vs Range ---
    ax4 = subplot(4, 1, 1); hold on; grid on; % ax4에 핸들 저장
    h_lines2(i) = plot(data.Acc_vs_Range.r, data.Acc_vs_Range.acc, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('4. Acc vs Range'); xlabel('Range (m)'); ylabel('Acceleration (m/s^2)');
    set(gca, 'XDir', 'reverse'); 
    
    % --- 5. LOS Angle ---
    ax5 = subplot(4, 1, 2); hold on; grid on; % ax5에 핸들 저장
    plot(t, data.LOS_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('5. LOS Angle'); xlabel('Time (s)'); ylabel('\lambda (deg)');
    
    % --- 6. Pursuer Lead Angle ---
    ax6 = subplot(4, 1, 3); hold on; grid on; % ax6에 핸들 저장
    plot(t, data.Pursuer_Lead_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('6. Pursuer Lead Angle'); xlabel('Time (s)'); ylabel('\sigma_p (deg)');
    
    % --- 7. Pursuer Heading Angle ---
    ax7 = subplot(4, 1, 4); hold on; grid on; % ax7에 핸들 저장
    plot(t, data.Pursuer_Heading_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('7. Pursuer Heading Angle'); xlabel('Time (s)'); ylabel('\psi_p (deg)');
end

% -----------------------------------------------------------
% 4. 범례(Legend) 추가
% -----------------------------------------------------------
valid_idx = (h_traj ~= 0);
if any(valid_idx)
    figure(fig_traj);
    legend(h_traj(valid_idx), legend_labels(valid_idx), 'Location', 'best');
    
    figure(fig1);  subplot(3, 1, 1);
    legend(h_lines1(valid_idx), legend_labels(valid_idx), 'Location', 'best');
    
    figure(fig2);  subplot(4, 1, 1);
    legend(h_lines2(valid_idx), legend_labels(valid_idx), 'Location', 'best');
end

% -----------------------------------------------------------
% 5. 자동 저장 (Auto Save) 기능 - 개별 Subplot 저장
% -----------------------------------------------------------
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% Axes 핸들을 넘겨주면 주변 여백을 알아서 계산해서 딱 해당 그래프만 저장해줘
exportgraphics(ax0, fullfile(save_dir, '0_Trajectory.png'), 'Resolution', 300);
exportgraphics(ax1, fullfile(save_dir, '1_Relative_Distance.png'), 'Resolution', 300);
exportgraphics(ax2, fullfile(save_dir, '2_Closing_Velocity.png'), 'Resolution', 300);
exportgraphics(ax3, fullfile(save_dir, '3_Pursuer_Acc.png'), 'Resolution', 300);
exportgraphics(ax4, fullfile(save_dir, '4_Acc_vs_Range.png'), 'Resolution', 300);
exportgraphics(ax5, fullfile(save_dir, '5_LOS_Angle.png'), 'Resolution', 300);
exportgraphics(ax6, fullfile(save_dir, '6_Pursuer_Lead_Angle.png'), 'Resolution', 300);
exportgraphics(ax7, fullfile(save_dir, '7_Pursuer_Heading_Angle.png'), 'Resolution', 300);

% 혹시 통째로 묶인 창도 보고서나 슬라이드에 쓸지 몰라서 주석 처리 안 하고 남겨뒀어
% exportgraphics(fig1, fullfile(save_dir, 'Combined_States_1.png'), 'Resolution', 300);
% exportgraphics(fig2, fullfile(save_dir, 'Combined_States_2.png'), 'Resolution', 300);

fprintf('으헤~ 개별 subplot들까지 전부 따로 분리해서 PNG로 구워뒀어 선생. 확인해봐~\n');