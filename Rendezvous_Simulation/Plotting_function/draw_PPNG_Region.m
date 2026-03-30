function draw_PPNG_Region(ax, sigma_p0, r_allow, N)
    % 으헤~ 선생님이 원한 대로 '반투명 초록색' 영역을 그리는 함수야.
    % Input:
    %   ax: 그릴 타일(Axes) 핸들
    %   sigma_p0: 현재 초기 리드각 [rad]
    %   r_allow: 최대 도달 가능 거리 [m]
    %   N: 유도 상수
    
    % Target 정보 (북쪽 고정)
    Xt = 0; Yt = 0;
    psi_t_rad = pi/2; 
    
    % --- (1) Independent Variable: sigma_tf ---
    sigma_tf_min = sigma_p0 / N;
    sigma_tf_max = pi/2;
    
    % 유효하지 않은 영역이면 그리지 않고 리턴
    if sigma_tf_min >= sigma_tf_max
        return; 
    end
    
    sigma_tf = linspace(sigma_tf_min, sigma_tf_max, 1000); 
    
    % --- (2) Dependent Variable: sigma_t0 ---
    sigma_t0 = (N * sigma_tf - sigma_p0) / (N - 1);
    
    % --- (3) Boundary Distance Calculation ---
    sigma_pf = sigma_tf; 
    numerator = cos(sigma_pf);
    denom_inner = (N * sigma_tf + (N - 2) * sigma_p0) / (2 * (N - 1));
    denominator = cos(denom_inner);
    exponent = 2 / (2 - N);
    
    % 0으로 나누기 방지 (분모가 0에 매우 가까우면 inf 처리)
    r0_boundary = r_allow * (numerator ./ denominator) .^ exponent;
    
    % --- (4) 좌표 변환 ---
    % Angle = Target Heading(90) + 180(Back) - sigma_t0(Left Spread)
    plot_angle_vec = psi_t_rad + pi - sigma_t0;  
    
    Region_X = Xt + r0_boundary .* cos(plot_angle_vec);
    Region_Y = Yt + r0_boundary .* sin(plot_angle_vec);
    
    % --- (5) 그리기 (초록색 설정) ---
    % [Xt, Region_X, Xt] -> 원점에서 시작해서 곡선 돌고 원점으로 닫힘 (부채꼴)
    % FaceColor: 'g' (초록색)
    % FaceAlpha: 0.3 (반투명)
    fill(ax, [Xt, Region_X, Xt], [Yt, Region_Y, Yt], 'g', ...
        'FaceAlpha', 0.3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % 경계선 그리기 (진한 초록색 실선)
    s_deg = round(rad2deg(sigma_p0)); 
    plot(ax, Region_X, Region_Y, 'Color', [0 0.5 0], 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Theoretical Limit'));
end