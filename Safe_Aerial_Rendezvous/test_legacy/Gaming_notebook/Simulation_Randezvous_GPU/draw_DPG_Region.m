function draw_DPG_Region(ax, sigma_p0, r_allow)
    % 으헤~ DPG는 리드각 유지가 핵심이지.
    
    Xt = 0; Yt = 0;
    psi_t_rad = pi/2;

    % --- (1) sigma_pf = sigma_p0 ---
    sigma_pf = sigma_p0;        
    
    % --- (2) sigma_t0 범위 ---
    sigma_t0_min = sigma_pf;
    sigma_t0_max = pi - sigma_pf;
    
    if sigma_t0_min >= sigma_t0_max
        return; 
    end

    sigma_t0 = linspace(sigma_t0_min, sigma_t0_max - 0.01, 500); 
    
    % --- (3) Reachable Region 계산 ---
    numerator = r_allow * cos(sigma_pf)^2;
    denominator = cos((sigma_t0 + sigma_pf)/2).^2;
    r_boundary = numerator ./ denominator; 
    
    % --- (4) 좌표 변환 ---
    plot_angle_vec = psi_t_rad + pi - sigma_t0; 
    Region_X = Xt + r_boundary .* cos(plot_angle_vec);
    Region_Y = Yt + r_boundary .* sin(plot_angle_vec);
    
    % --- (5) 그리기 ---
    fill(ax, [Xt, Region_X, Xt], [Yt, Region_Y, Yt], 'y', ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    plot(ax, Region_X, Region_Y, 'y--', 'LineWidth', 1.5, ...
        'DisplayName', 'Theory Boundary');
end