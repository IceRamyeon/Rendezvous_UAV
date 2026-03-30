function Max_Acc(cfg, sigma_input)
    % Max_Acc: 현재 리드각(sigma_p)을 유지할 때 가속도 제한에 걸리는 거리를 계산해서 그립니다.
    % 
    % 입력:
    %   cfg         : main_rand 설정 구조체 (V_p, limit_acc 사용)
    %   sigma_input : 현재 Pursuer Lead Angle (rad 단위 권장)
    %
    % 조건: sigma_ref = sigma_p (피드백 제어량 0 가정, 순수 기하학적 포화)
    
    % 1. 파라미터 추출
    V = cfg.V_p;
    acc_limit = cfg.limit_acc * 9.81; % G -> m/s^2 변환
    
    sigma_p = deg2rad(abs(sigma_input)); % 대칭성을 위해 절댓값 사용
    
    % 2. 해석적 경계 계산 (Analytic Boundary)
    % 공식: Acc = (V^2 / r) * (sin(sigma_t) - sin(sigma_p))
    % 따라서 r_limit = (V^2 / acc_limit) * |sin(sigma_t) - sin(sigma_p)|
    
    % sigma_t를 0 ~ 360도 한 바퀴 돌림
    sigma_t_vec = linspace(0, 2*pi, 360);
    
    % 경계 거리(Radius) 계산
    r_boundary = (V^2 / acc_limit) * abs(sin(sigma_t_vec) - sin(sigma_p));
    
    % 3. 좌표 변환 (Polar -> Cartesian)
    % Target Centered Frame (Target Heading = 90 deg 고정)
    % P의 위치 각도 = (Target Heading) - sigma_t + 180도
    psi_t = pi/2;
    plot_angle_vec = psi_t - sigma_t_vec + pi;
    
    Boundary_X = r_boundary .* cos(plot_angle_vec);
    Boundary_Y = r_boundary .* sin(plot_angle_vec);
    
    % 4. 그리기 (Figure 3 위에 덧칠)
    figure(3);
    hold on;
    
    % [추가된 기능] 기존에 그렸던 객체들 찾아서 지우기 (애니메이션 효과)
    % 'Tag'를 이용해 이 함수가 그린 것들만 콕 집어서 삭제함
    delete(findobj(gca, 'Tag', 'MaxAccFill'));
    delete(findobj(gca, 'Tag', 'MaxAccLine'));
    delete(findobj(gca, 'Tag', 'MaxAccText'));
    
    % (1) 위험 영역(Saturation Region) 색칠 - 빨간색 반투명
    % 원점(0,0)과 경계선 사이가 위험한 영역(r < r_boundary)임
    fill(Boundary_X, Boundary_Y, 'r', ...
        'FaceAlpha', 0.1, 'EdgeColor', 'none', ...
        'DisplayName', 'Saturation Zone', 'HandleVisibility', 'off', ...
        'Tag', 'MaxAccFill'); % 태그 추가
        
    % (2) 경계선 그리기 - 빨간색 실선
    plot(Boundary_X, Boundary_Y, 'r-', 'LineWidth', 2, ...
        'DisplayName', sprintf('Acc Limit (\\sigma_p=%.0f^\\circ)', sigma_p*180/pi), ...
        'Tag', 'MaxAccLine'); % 태그 추가
    
    % (3) 텍스트 표시 (선택 사항)
    % 가장 튀어나온 부분에 라벨 달기
    [max_r, idx] = max(r_boundary);
    if max_r > 10 % 너무 작으면 글씨 안 씀
        text(Boundary_X(idx), Boundary_Y(idx), '  Max Acc Limit', ...
            'Color', 'r', 'FontSize', 8, 'VerticalAlignment', 'bottom', ...
            'Tag', 'MaxAccText'); % 태그 추가
    end
end