%% Plot_Multiple_Scenarios.m
% main.m에서 target_path에 저장된 matlab 파일을 입력 받아 다수의 plot을 한꺼번에 plot하는 매트랩 파일
clc; clear; close all;

% Target 정보 입력
xt = 0;            % [m]
yt = 0;           % [m]
heading = 90;      % [deg]


% -----------------------------------------------------------
% 1. 플롯할 파일 이름 배열 설정 (여기에 파일명 추가/삭제)
% -----------------------------------------------------------
file_list = {
    'FRS_-30.mat', ...
    'FRS_0.mat', ...
    'FRS_30.mat'
};

% List of comparison files(비워두면 file_list만 단독으로 플롯함)
file_list_comparison = {
};

% proposed method(line)
custom_names_prop = {'UAV1 (\sigma_{p0}=-30\circ, Proposed)', ...
                     'UAV2 (\sigma_{p0}=0\circ, Proposed)', ...
                     'UAV3 (\sigma_{p0}=30\circ, Proposed)'};

% comparison(dash-line)
custom_names_comp = {'UAV1 (\sigma_{p0}=40\circ, RDPG)', ...
                     'UAV2 (\sigma_{p0}=60\circ, RDPG)', ... 
                     'UAV3 (\sigma_{p0}=80\circ, RDPG)'};
% -----------------------------------------------------------

% 파일이 저장된 폴더 경로
data_dir = 'C:\Users\최혁재\OneDrive\Desktop\AISL 자료\미팅 자료\0724 랩미팅\Sim4.4';

% 자동으로 PNG를 저장할 폴더 경로
save_dir = 'C:\Users\최혁재\OneDrive\Desktop\AISL 자료\미팅 자료\0724 랩미팅\Sim4.4\plots';

% -----------------------------------------------------------
% 2. 플롯 및 색상/마커 초기화
% -----------------------------------------------------------
colors = lines(length(file_list)); 
markers = {'o', 's', '*'}; % 동그라미, 네모, 별표 순환

% 비교 모드 활성화 여부 체크
is_comparison = ~isempty(file_list_comparison);

% 창 분할: 이미지 비율처럼 넓적하게 설정
fig_traj = figure('Name', 'Trajectory Analysis', 'Position', [100, 100, 800, 600], 'Theme', 'light'); 
fig1     = figure('Name', 'Rendezvous Simulation Results (1/2)', 'Position', [150, 150, 1000, 800], 'Theme', 'light');
fig2     = figure('Name', 'Rendezvous Simulation Results (2/2)', 'Position', [200, 200, 1000, 800], 'Theme', 'light');

% -----------------------------------------------------------
% 3. Axes 미리 세팅 (각 Subplot 핸들 지정)
% -----------------------------------------------------------
figure(fig_traj);
ax0 = gca; hold on; grid on; xlabel('X (m)'); ylabel('Y (m)'); axis equal;

figure(fig1);
ax1 = subplot(3, 1, 1); hold on; grid on; xlabel('Time (s)'); ylabel('Range (m)');
ax2 = subplot(3, 1, 2); hold on; grid on; xlabel('Time (s)'); ylabel('V_c (m/s)');
ax3 = subplot(3, 1, 3); hold on; grid on; xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');

figure(fig2);
ax4 = subplot(4, 1, 1); hold on; grid on; xlabel('Range (m)'); ylabel('Acceleration (m/s^2)'); set(gca, 'XDir', 'reverse'); 
ax5 = subplot(4, 1, 2); hold on; grid on; xlabel('Time (s)'); ylabel('\lambda (deg)');
ax6 = subplot(4, 1, 3); hold on; grid on; xlabel('Time (s)'); ylabel('\sigma_p (deg)');
ax7 = subplot(4, 1, 4); hold on; grid on; xlabel('Time (s)'); ylabel('\psi_p (deg)');

% --- 범례 핸들 저장을 위한 객체 배열 초기화 ---
n_files = length(file_list);
h0_c = gobjects(1, n_files); h0_p = gobjects(1, n_files);
h1_c = gobjects(1, n_files); h1_p = gobjects(1, n_files);
h2_c = gobjects(1, n_files); h2_p = gobjects(1, n_files);
h3_c = gobjects(1, n_files); h3_p = gobjects(1, n_files);
h4_c = gobjects(1, n_files); h4_p = gobjects(1, n_files);
h5_c = gobjects(1, n_files); h5_p = gobjects(1, n_files);
h6_c = gobjects(1, n_files); h6_p = gobjects(1, n_files);
h7_c = gobjects(1, n_files); h7_p = gobjects(1, n_files);

% -----------------------------------------------------------
% 4. 반복문을 통한 데이터 로드 및 플로팅
% -----------------------------------------------------------
for i = 1:n_files
    % --- 기본 데이터 로드 ---
    full_path_prop = fullfile(data_dir, file_list{i});
    if ~exist(full_path_prop, 'file')
        warning('%s 파일이 없음. 건너뜀 >>', file_list{i});
        continue;
    end
    data_prop = load(full_path_prop).log_data;
    t_prop = data_prop.Time;
    
    c_style = colors(i,:);
    m_style = markers{mod(i-1, length(markers)) + 1}; % 마커 순환
    
    mk_idx_prop = round(linspace(1, length(t_prop), 15));
    idx_r_prop  = round(linspace(1, length(data_prop.Acc_vs_Range.r), 15));
    
    % --- 비교군 데이터 로드 (비교 모드일 때만 실행) ---
    if is_comparison
        full_path_rdpg = fullfile(data_dir, file_list_comparison{i});
        if ~exist(full_path_rdpg, 'file')
            warning('으헤... 비교군 %s 파일이 없어. 이 파일은 건너뛸게.', file_list_comparison{i});
            continue;
        end
        data_rdpg = load(full_path_rdpg).log_data;
        t_rdpg = data_rdpg.Time;
        
        mk_idx_rdpg = round(linspace(1, length(t_rdpg), 15));
        idx_r_rdpg  = round(linspace(1, length(data_rdpg.Acc_vs_Range.r), 15));
        
        % RDPG 플롯 (점선) - 핸들 저장
        h0_c(i) = plot(ax0, data_rdpg.Trajectory.Xp, data_rdpg.Trajectory.Yp, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h1_c(i) = plot(ax1, t_rdpg, data_rdpg.Relative_Distance, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h2_c(i) = plot(ax2, t_rdpg, data_rdpg.Closing_Velocity, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h3_c(i) = plot(ax3, t_rdpg, data_rdpg.Pursuer_Acc, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h4_c(i) = plot(ax4, data_rdpg.Acc_vs_Range.r, data_rdpg.Acc_vs_Range.acc, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', idx_r_rdpg, 'LineWidth', 1.5);
        h5_c(i) = plot(ax5, t_rdpg, data_rdpg.LOS_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h6_c(i) = plot(ax6, t_rdpg, data_rdpg.Pursuer_Lead_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
        h7_c(i) = plot(ax7, t_rdpg, data_rdpg.Pursuer_Heading_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5);
    end
    
    % ==========================================
    % 기본/Proposed 데이터 플롯 (실선)
    % ==========================================
    plot(ax0, data_prop.Trajectory.Xt, data_prop.Trajectory.Yt, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off'); %[cite: 2]
    
    % Trajectory Plot에 Reachable Region 그리기 (heading은 라디안으로 변환)
    % psi_t_rad = deg2rad(heading);
    % draw_ReachableRegion_forMultipleplots(ax0, 20, 1, 2, xt, yt, psi_t_rad);
    % -------------------
    
    % Plot 그리기
    h0_p(i) = plot(ax0, data_prop.Trajectory.Xp, data_prop.Trajectory.Yp, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h1_p(i) = plot(ax1, t_prop, data_prop.Relative_Distance, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h2_p(i) = plot(ax2, t_prop, data_prop.Closing_Velocity, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h3_p(i) = plot(ax3, t_prop, data_prop.Pursuer_Acc, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h4_p(i) = plot(ax4, data_prop.Acc_vs_Range.r, data_prop.Acc_vs_Range.acc, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', idx_r_prop, 'LineWidth', 1.5);
    h5_p(i) = plot(ax5, t_prop, data_prop.LOS_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h6_p(i) = plot(ax6, t_prop, data_prop.Pursuer_Lead_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
    h7_p(i) = plot(ax7, t_prop, data_prop.Pursuer_Heading_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5);
end

% -----------------------------------------------------------
% 5. 범례(Legend) 추가 및 2열 정렬
% -----------------------------------------------------------
if is_comparison
    % 비교군 핸들과 제안 핸들을 합치고, 2열로 배치
    lgd_handles_0 = [h0_c, h0_p]; lgd_handles_1 = [h1_c, h1_p];
    lgd_handles_2 = [h2_c, h2_p]; lgd_handles_3 = [h3_c, h3_p];
    lgd_handles_4 = [h4_c, h4_p]; lgd_handles_5 = [h5_c, h5_p];
    lgd_handles_6 = [h6_c, h6_p]; lgd_handles_7 = [h7_c, h7_p];
    
    lgd_labels = [custom_names_comp, custom_names_prop];
    cols = 2;
else
    % 비교군이 없으면 Proposed만 플롯
    lgd_handles_0 = h0_p; lgd_handles_1 = h1_p;
    lgd_handles_2 = h2_p; lgd_handles_3 = h3_p;
    lgd_handles_4 = h4_p; lgd_handles_5 = h5_p;
    lgd_handles_6 = h6_p; lgd_handles_7 = h7_p;
    
    lgd_labels = custom_names_prop;
    cols = 1;
end

legend(ax0, lgd_handles_0, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax1, lgd_handles_1, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax2, lgd_handles_2, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax3, lgd_handles_3, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax4, lgd_handles_4, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax5, lgd_handles_5, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax6, lgd_handles_6, lgd_labels, 'NumColumns', cols, 'Location', 'best');
legend(ax7, lgd_handles_7, lgd_labels, 'NumColumns', cols, 'Location', 'best');

% -----------------------------------------------------------
% 6. 자동 저장 (Auto Save) 기능
% -----------------------------------------------------------
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

exportgraphics(ax0, fullfile(save_dir, '0_Trajectory.png'), 'Resolution', 300);
exportgraphics(ax1, fullfile(save_dir, '1_Relative_Distance.png'), 'Resolution', 300);
exportgraphics(ax2, fullfile(save_dir, '2_Closing_Velocity.png'), 'Resolution', 300);
exportgraphics(ax3, fullfile(save_dir, '3_Pursuer_Acc.png'), 'Resolution', 300);
exportgraphics(ax4, fullfile(save_dir, '4_Acc_vs_Range.png'), 'Resolution', 300);
exportgraphics(ax5, fullfile(save_dir, '5_LOS_Angle.png'), 'Resolution', 300);
exportgraphics(ax6, fullfile(save_dir, '6_Pursuer_Lead_Angle.png'), 'Resolution', 300);
exportgraphics(ax7, fullfile(save_dir, '7_Pursuer_Heading_Angle.png'), 'Resolution', 300);

fprintf('<< Multiple Plots 생성 완료 >>\n');