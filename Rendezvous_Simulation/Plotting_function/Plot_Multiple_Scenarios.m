%% Plot_Multiple_Scenarios.m
clc; clear; close all;

% -----------------------------------------------------------
% 1. 플롯할 파일 이름 배열 설정 (여기에 파일명 추가/삭제)
% -----------------------------------------------------------
file_list = {
    'Scenario_SigmaP0_0.mat', ...
    'Scenario_SigmaP0_20.mat', ...
    'Scenario_SigmaP0_40.mat'
};

% 파일이 저장된 폴더 경로 (선생이 쓰는 경로로 수정해)
data_dir = 'C:\Users\jedie\OneDrive\문서\대학 자료\AISL 연구실\미팅 및 발표 자료\260629 랩미팅 준비\2\Theorem1_Verification_1G_Clip';

% -----------------------------------------------------------
% 2. 플롯 및 색상 초기화
% -----------------------------------------------------------
% 파일 개수만큼 서로 다른 색상 자동 할당
colors = lines(length(file_list)); 

% 1600x900 사이즈로 큼지막한 창 하나 생성
figure('Name', 'Rendezvous Simulation Results', 'Position', [100, 100, 1600, 900]);

% 범례(Legend) 핸들과 라벨을 저장할 빈 배열
h_lines = zeros(1, length(file_list));
legend_labels = cell(1, length(file_list));

% -----------------------------------------------------------
% 3. 반복문을 통한 데이터 로드 및 플로팅
% -----------------------------------------------------------
for i = 1:length(file_list)
    full_path = fullfile(data_dir, file_list{i});
    
    % 파일 존재 여부 확인
    if ~exist(full_path, 'file')
        warning('으헤... %s 파일이 지정된 경로에 없어. 건너뛸게.', file_list{i});
        continue;
    end
    
    % 데이터 로드
    loaded = load(full_path);
    data = loaded.log_data;
    t = data.Time;
    
    % 파일명에서 확장자를 빼고 '_' 문자를 텍스트에서 안 깨지게 치환해서 범례로 사용
    [~, name, ~] = fileparts(file_list{i});
    legend_labels{i} = strrep(name, '_', '\_');
    
    % --- 0. Trajectory ---
    subplot(2, 4, 1); hold on; grid on;
    % 타겟 궤적은 검은색 점선 (모든 시나리오가 같다고 가정하고 덮어씀)
    plot(data.Trajectory.Xt, data.Trajectory.Yt, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off'); 
    h_lines(i) = plot(data.Trajectory.Xp, data.Trajectory.Yp, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('0. Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal;
    
    % --- 1. Relative Distance ---
    subplot(2, 4, 2); hold on; grid on;
    plot(t, data.Relative_Distance, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('1. Relative Distance'); xlabel('Time (s)'); ylabel('Range (m)');
    
    % --- 2. Closing Velocity ---
    subplot(2, 4, 3); hold on; grid on;
    plot(t, data.Closing_Velocity, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('2. Closing Velocity'); xlabel('Time (s)'); ylabel('V_c (m/s)');
    
    % --- 3. Pursuer Acc ---
    subplot(2, 4, 4); hold on; grid on;
    plot(t, data.Pursuer_Acc, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('3. Pursuer Acc'); xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
    
    % --- 4. Acc vs Range ---
    % 거리가 줄어드는 방향(랑데부)이니까 x축을 반전시키면 시간 흐름대로 보기 편해
    subplot(2, 4, 5); hold on; grid on;
    plot(data.Acc_vs_Range.r, data.Acc_vs_Range.acc, 'Color', colors(i,:), 'LineWidth', 1.5);
    title('4. Acc vs Range'); xlabel('Range (m)'); ylabel('Acceleration (m/s^2)');
    set(gca, 'XDir', 'reverse'); 
    
    % --- 5. LOS Angle ---
    subplot(2, 4, 6); hold on; grid on;
    plot(t, data.LOS_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('5. LOS Angle'); xlabel('Time (s)'); ylabel('\lambda (deg)');
    
    % --- 6. Pursuer Lead Angle ---
    subplot(2, 4, 7); hold on; grid on;
    plot(t, data.Pursuer_Lead_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('6. Pursuer Lead Angle'); xlabel('Time (s)'); ylabel('\sigma_p (deg)');
    
    % --- 7. Pursuer Heading Angle ---
    subplot(2, 4, 8); hold on; grid on;
    plot(t, data.Pursuer_Heading_Angle * (180/pi), 'Color', colors(i,:), 'LineWidth', 1.5);
    title('7. Pursuer Heading Angle'); xlabel('Time (s)'); ylabel('\psi_p (deg)');
end

% -----------------------------------------------------------
% 4. 범례(Legend) 추가
% -----------------------------------------------------------
% 궤적 그래프에만 한 번 띄워서 공간 확보 (h_lines에 유효한 핸들이 있는 것만 묶음)
valid_idx = (h_lines ~= 0);
if any(valid_idx)
    subplot(2, 4, 1);
    legend(h_lines(valid_idx), legend_labels(valid_idx), 'Location', 'best');
end