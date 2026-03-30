    function RR_DPG(cfg, sigma_input)
    % RR_DPG: 설정(cfg)과 기준 리드각(sigma_input)을 받아 DPG 영역을 그리는 함수
    % 
    % 입력:
    %   cfg         : main_rand에서 설정한 파라미터 구조체 (r_allow 등 포함)
    %   sigma_input : Region을 정의할 기준 Lead Angle (단위: Radian 권장)
    
    % 1. 파라미터 추출
    r_allow = cfg.r_allow;
    
    % 타겟 위치 (Target Centered Frame 기준: 0,0)
    Xt = 0; Yt = 0;
    
    % 타겟 헤딩 (Figure 3는 North(90도) 고정)
    psi_t_rad = pi/2; 

    % 2. 입력된 sigma_ref 처리 (절댓값 사용)
    % 선생이 sigma_p를 넣는다고 했으니, 혹시 음수일 수도 있어서 절댓값 취함.
    % (DPG 영역은 좌우 대칭이니까!)
    
    sigma_pf = deg2rad(abs(sigma_input)); 
    
    % 만약 입력값이 너무 크면(ex: 30도 -> 30), Radian으로 변환 안 된 걸로 간주
    if sigma_pf > 2*pi
        sigma_pf = sigma_pf * (pi/180);
    end

    % 3. sigma_t0 범위 설정 (이론적 한계)
    sigma_t0_min = sigma_pf;
    sigma_t0_max = pi - sigma_pf;
    
    % 그릴 수 없는 조건이면 리턴
    if sigma_t0_min >= sigma_t0_max
        fprintf('>>> [RR_DPG] 그릴 수 없는 영역 (Sigma_ref가 너무 큼)\n');
        return; 
    end

    % 4. Reachable Region 경계 계산
    % 매끄러운 곡선을 위해 점 500개 찍기
    sigma_t0 = linspace(sigma_t0_min, sigma_t0_max - 0.01, 500); 
    
    numerator = r_allow * cos(sigma_pf)^2;
    denominator = cos((sigma_t0 + sigma_pf)/2).^2;
    r_boundary = numerator ./ denominator; 
    
    % 5. 좌표 변환 (Polar -> Cartesian)
    % 타겟 헤딩(90도)을 기준으로 상대각도 계산
    plot_angle_vec = psi_t_rad + pi - sigma_t0; 
    
    Region_X = Xt + r_boundary .* cos(plot_angle_vec);
    Region_Y = Yt + r_boundary .* sin(plot_angle_vec);
    
    % 6. 그리기 (Figure 3 위에 덧칠)
    figure(3);
    hold on;
    
    % (1) 영역 채우기 (노란색, 투명도 0.15)
    fill([Xt, Region_X, Xt], [Yt, Region_Y, Yt], 'y', ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % (2) 경계선 그리기 (점선)
    plot(Region_X, Region_Y, 'y--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Region (\\sigma=%.1f^\\circ)', sigma_pf*180/pi));
    
    % (3) 반대편 대칭 영역도 그릴까? (필요하면 주석 해제해)
    % plot_angle_vec_sym = psi_t_rad - (pi - sigma_t0);
    % Region_X_sym = Xt + r_boundary .* cos(plot_angle_vec_sym);
    % Region_Y_sym = Yt + r_boundary .* sin(plot_angle_vec_sym);
    % fill([Xt, Region_X_sym, Xt], [Yt, Region_Y_sym, Yt], 'y', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    % plot(Region_X_sym, Region_Y_sym, 'y--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

end