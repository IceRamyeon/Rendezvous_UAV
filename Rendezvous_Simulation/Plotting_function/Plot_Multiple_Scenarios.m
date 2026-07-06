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

% RDPG 비교군 파일 리스트 (비워두면 file_list만 단독으로 플롯함)
file_list_comparison = {

};

% 파일이 저장된 폴더 경로
data_dir = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260703 미팅 준비\Sim4.3';
% 자동으로 PNG를 저장할 폴더 경로
save_dir = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260703 미팅 준비\Sim4.3';

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
ax0 = gca; hold on; grid on;
title('0. Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal;

figure(fig1);
ax1 = subplot(3, 1, 1); hold on; grid on; title('1. Relative Distance'); xlabel('Time (s)'); ylabel('Range (m)');
ax2 = subplot(3, 1, 2); hold on; grid on; title('2. Closing Velocity'); xlabel('Time (s)'); ylabel('V_c (m/s)');
ax3 = subplot(3, 1, 3); hold on; grid on; title('3. Pursuer Acc'); xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');

figure(fig2);
ax4 = subplot(4, 1, 1); hold on; grid on; title('4. Acc vs Range'); xlabel('Range (m)'); ylabel('Acceleration (m/s^2)'); set(gca, 'XDir', 'reverse'); 
ax5 = subplot(4, 1, 2); hold on; grid on; title('5. LOS Angle'); xlabel('Time (s)'); ylabel('\lambda (deg)');
ax6 = subplot(4, 1, 3); hold on; grid on; title('6. Pursuer Lead Angle'); xlabel('Time (s)'); ylabel('\sigma_p (deg)');
ax7 = subplot(4, 1, 4); hold on; grid on; title('7. Pursuer Heading Angle'); xlabel('Time (s)'); ylabel('\psi_p (deg)');

% -----------------------------------------------------------
% 4. 반복문을 통한 데이터 로드 및 플로팅
% -----------------------------------------------------------
for i = 1:length(file_list)
    % --- 기본 데이터 로드 ---
    full_path_prop = fullfile(data_dir, file_list{i});
    if ~exist(full_path_prop, 'file')
        warning('으헤... %s 파일이 없어. 건너뛸게.', file_list{i});
        continue;
    end
    data_prop = load(full_path_prop).log_data;
    t_prop = data_prop.Time;
    
    % 이름 및 스타일 설정
    [~, name_prop, ~] = fileparts(file_list{i});
    name_prop = strrep(name_prop, '_', '\_');
    
    c_style = colors(i,:);
    m_style = markers{mod(i-1, length(markers)) + 1}; % 마커 순환
    
    mk_idx_prop = round(linspace(1, length(t_prop), 15));
    idx_r_prop  = round(linspace(1, length(data_prop.Acc_vs_Range.r), 15));
    
    % 범례에 표시할 이름 결정
    if is_comparison
        display_name_prop = [name_prop, ' (Proposed)'];
    else
        display_name_prop = name_prop;
    end
    
    % --- 비교군 데이터 로드 (비교 모드일 때만 실행) ---
    if is_comparison
        full_path_rdpg = fullfile(data_dir, file_list_comparison{i});
        if ~exist(full_path_rdpg, 'file')
            warning('으헤... 비교군 %s 파일이 없어. 이 파일은 건너뛸게.', file_list_comparison{i});
            continue;
        end
        data_rdpg = load(full_path_rdpg).log_data;
        t_rdpg = data_rdpg.Time;
        
        [~, name_rdpg, ~] = fileparts(file_list_comparison{i});
        name_rdpg = strrep(name_rdpg, '_', '\_');
        mk_idx_rdpg = round(linspace(1, length(t_rdpg), 15));
        idx_r_rdpg  = round(linspace(1, length(data_rdpg.Acc_vs_Range.r), 15));
        
        % RDPG 플롯 (점선)
        plot(ax0, data_rdpg.Trajectory.Xp, data_rdpg.Trajectory.Yp, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax1, t_rdpg, data_rdpg.Relative_Distance, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax2, t_rdpg, data_rdpg.Closing_Velocity, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax3, t_rdpg, data_rdpg.Pursuer_Acc, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax4, data_rdpg.Acc_vs_Range.r, data_rdpg.Acc_vs_Range.acc, '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', idx_r_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax5, t_rdpg, data_rdpg.LOS_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax6, t_rdpg, data_rdpg.Pursuer_Lead_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
        plot(ax7, t_rdpg, data_rdpg.Pursuer_Heading_Angle * (180/pi), '--', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_rdpg, 'LineWidth', 1.5, 'DisplayName', [name_rdpg, ' (RDPG)']);
    end
    
    % ==========================================
    % 기본/Proposed 데이터 플롯 (실선)
    % ==========================================
    plot(ax0, data_prop.Trajectory.Xt, data_prop.Trajectory.Yt, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off'); 
    plot(ax0, data_prop.Trajectory.Xp, data_prop.Trajectory.Yp, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    
    plot(ax1, t_prop, data_prop.Relative_Distance, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    plot(ax2, t_prop, data_prop.Closing_Velocity, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    plot(ax3, t_prop, data_prop.Pursuer_Acc, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    
    plot(ax4, data_prop.Acc_vs_Range.r, data_prop.Acc_vs_Range.acc, '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', idx_r_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    plot(ax5, t_prop, data_prop.LOS_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    plot(ax6, t_prop, data_prop.Pursuer_Lead_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
    plot(ax7, t_prop, data_prop.Pursuer_Heading_Angle * (180/pi), '-', 'Color', c_style, 'Marker', m_style, 'MarkerIndices', mk_idx_prop, 'LineWidth', 1.5, 'DisplayName', display_name_prop);
end

% -----------------------------------------------------------
% 5. 범례(Legend) 추가
% -----------------------------------------------------------
legend(ax0, 'Location', 'best');
legend(ax1, 'Location', 'best');
legend(ax2, 'Location', 'best');
legend(ax3, 'Location', 'best');
legend(ax4, 'Location', 'best');
legend(ax5, 'Location', 'best');
legend(ax6, 'Location', 'best');
legend(ax7, 'Location', 'best');

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

fprintf('으헤~ 세팅 완료했어. 확인해봐 선생~\n');