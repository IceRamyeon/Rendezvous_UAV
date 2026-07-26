function Trace_DPG_Region(sigma_p, r_f, fig_num)
    % 으헤~ DPG는 리드각 유지가 핵심이지.
    %
    % [사용법 1: 자동으로 Figure 3에 그리기]
    % Trace_DPG_Region(sigma_p, r_f)
    %
    % [사용법 2: 특정 축(ax) 지정해서 그리기]
    % Trace_DPG_Region(ax, sigma_p, r_f)
    close all;
    
    %% 1. 입력 변수 처리
    if nargin == 2
        % 입력이 2개면 -> 자동으로 Figure 3을 타겟으로 설정
        figure(3);
        ax = gca;
        sigma_p0 = sigma_p;
        r_allow = r_f;
    elseif nargin == 3
        % 입력이 3개면 -> 입력받은 ax 사용
        ax = fig_num;
        sigma_p0 = sigma_p;
        r_allow = r_f;
    else
        error('입력 변수는 2개(자동) 또는 3개(수동)여야 해');
    end

    %% 2. 파라미터 및 좌표계 설정
    Xt = 0; Yt = 0;
    
    % 리드각은 절댓값으로 처리해서 좌우 대칭으로 그릴 거야
    sigma_pf = deg2rad(sigma_p0);        
    
    % --- (1) sigma_t0 범위 ---
    sigma_t0_min = sigma_pf;
    sigma_t0_max = pi - sigma_pf;
    
    if sigma_t0_min >= sigma_t0_max
        fprintf('>>> [Trace_DPG_Region] 으엥? sigma_pf가 너무 커서 도달 가능 영역이 없어!\n');
        return; 
    end

    % 끝부분에서 무한대로 발산(분모=0)하는 걸 막기 위해 여백을 살짝 줌
    sigma_t0 = linspace(sigma_t0_min, sigma_t0_max - 0.005, 500); 
    
    % --- (2) Reachable Region 거리 (r_boundary) 계산 ---
    numerator = r_allow * cos(sigma_pf)^2;
    denominator = cos((sigma_t0 + sigma_pf)/2).^2;
    r_boundary = numerator ./ denominator; 
    
    % 거리가 너무 커서 그래프 축이 망가지는 걸 방지 (최대 2000m 컷)
    r_boundary(r_boundary > 2000) = NaN;
    
    % --- (3) 좌표 변환 (Target Centered Frame) ---
    % Target은 (0,0)에 있고, +Y 방향(pi/2)을 바라봄
    plot_angle_right = -pi/2 - sigma_t0; % Target 기준 오른쪽
    plot_angle_left  = -pi/2 + sigma_t0; % Target 기준 왼쪽 (대칭)
    
    Region_X_R = Xt + r_boundary .* cos(plot_angle_right);
    Region_Y_R = Yt + r_boundary .* sin(plot_angle_right);
    
    Region_X_L = Xt + r_boundary .* cos(plot_angle_left);
    Region_Y_L = Yt + r_boundary .* sin(plot_angle_left);
    
    %% 3. 그리기 (투명도 적용된 굵은 빨간색 선)
    hold(ax, 'on'); 
    
    % 애니메이션이나 반복 실행 시 이전 선이 겹치지 않게 싹 지워주기
    delete(findobj(ax, 'Tag', 'DPGRegionLineR'));
    delete(findobj(ax, 'Tag', 'DPGRegionLineL'));

    % --- [투명도 50% (0.5) 적용! 으헤~] ---
    % 'Color', [1, 0, 0, 0.5] -> 빨간색(R=1)에 투명도 절반(0.5)이라는 뜻이야!
    plot(ax, Region_X_R, Region_Y_R, 'LineStyle', '-', 'LineWidth', 3, ...
        'Color', [0, 1, 0, 0.3], ... 
        'DisplayName', 'Reachable Region', 'Tag', 'DPGRegionLineR');
        
    plot(ax, Region_X_L, Region_Y_L, 'LineStyle', '-', 'LineWidth', 3, ...
        'Color', [0, 1, 0, 0.3], ... 
        'HandleVisibility', 'off', 'Tag', 'DPGRegionLineL');
end