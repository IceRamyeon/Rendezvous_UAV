function BRS_DPG(sigma_ref_deg, r_f)
    % BRS_DPG : DPG 유도 법칙 기반 도달 가능 영역(Backward Reachable Set) 시각화
    % 
    % [사용법]
    % 입력 변수 없이 BRS_DPG()만 실행하면 기본값으로 새 Figure에 그려줌!
    % 특정 값으로 그리고 싶으면 BRS_DPG(30, 2.0) 처럼 입력.

    close all;

    %% 1. 입력 변수 처리 및 초기화
    if nargin < 2
        r_f = 2.0; % [m] 허용 오차 거리 기본값
        if nargin < 1
            sigma_ref_deg = 60; % [deg] 기준 리드각 기본값
        end
        % 단독 실행 시 새 창 생성
        figure('Name', 'BRS for DPG', 'Theme', 'light', 'Position', [100, 100, 700, 600]);
        ax = gca;
    else
        ax = gca; % 외부에서 부를 때는 현재 활성화된 축 사용
    end

    %% 2. 파라미터 및 각도 설정
    sigma_ref = deg2rad(abs(sigma_ref_deg));
    
    % Target 중심 좌표계 설정
    Xt = 0; Yt = 0;
    
    %% 3. 도달 가능 거리(r_boundary) 계산
    % sigma_t 범위: sigma_ref 부터 pi - sigma_ref 까지
    sigma_t_min = sigma_ref;
    sigma_t_max = pi - sigma_ref;
    
    if sigma_t_min >= sigma_t_max
        fprintf('>>> 으엥? sigma_ref가 너무 커서 도달 가능 영역이 없어~\n');
        return;
    end
    
    % 끝단에서 무한대로 발산하는 것을 막기 위해 여백(0.005) 부여
    sigma_t = linspace(sigma_t_min, sigma_t_max - 0.005, 500);
    
    % 논문 수식 역산 적용
    numerator = r_f * cos(sigma_ref)^2;
    denominator = cos((sigma_t + sigma_ref)/2).^2;
    r_boundary = numerator ./ denominator;
    
    % 축이 우주 끝까지 가는 걸 막기 위한 안전장치
    r_boundary(r_boundary > 2000) = NaN;
    
    %% 4. 좌표 변환 (Bearing Angle 적용)
    % Target은 +y 방향(pi/2)을 바라봄. 
    % 방위각(Bearing) 개념을 적용해 Target 앞쪽을 기준으로 역산된 상대 각도 도출
    plot_angle_right = -pi/2 - sigma_t; % Target 앞쪽 오른쪽
    plot_angle_left  = -pi/2 + sigma_t; % Target 앞쪽 왼쪽 (좌우 대칭)
    
    X_R = Xt + r_boundary .* cos(plot_angle_right);
    Y_R = Yt + r_boundary .* sin(plot_angle_right);
    
    X_L = Xt + r_boundary .* cos(plot_angle_left);
    Y_L = Yt + r_boundary .* sin(plot_angle_left);
    
    %% 5. 영역 그리기
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    % 색칠된 BRS 영역 (연한 파란색)
    fill(ax, [Xt, X_R, Xt], [Yt, Y_R, Yt], 'b', ...
        'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    fill(ax, [Xt, X_L, Xt], [Yt, Y_L, Yt], 'b', ...
        'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
    % BRS 경계선 (파란색 굵은 선)
    plot(ax, X_R, Y_R, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Reachable Boundary');
    plot(ax, X_L, Y_L, 'b-', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % Target 화살표 및 위치 (빨간색)
    quiver(ax, Xt, Yt, 0, max(abs(Y_R))*0.15, 'r', 'LineWidth', 2, ...
        'MaxHeadSize', 0.5, 'DisplayName', 'Target Heading');
    plot(ax, Xt, Yt, 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    
    % 축 및 레이블 디자인
    title(ax, sprintf('Backward Reachable Set ($\\sigma_{ref} = %d^\\circ, r_f = %.1f$m)', ...
        sigma_ref_deg, r_f), 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel(ax, 'Relative x (m)');
    ylabel(ax, 'Relative y (m)');
    
    % 그래프 보기 좋게 범위 세팅
    % x_limit = max(abs(X_R)) * 1.1;
    xlim(ax, [-1000, 10]);
    ylim(ax, [-10, 1000]);
    
    legend(ax, 'Location', 'best');
end