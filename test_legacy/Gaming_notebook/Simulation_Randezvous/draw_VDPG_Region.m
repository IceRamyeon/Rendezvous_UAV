function draw_VDPG_Region(ax, sigma_p0, r_allow)
    % 으헤~ VDPG는 적분이 필요해서 조금 귀찮지만... 해보자구.
    
    Xt = 0; Yt = 0;
    psi_t_rad = pi/2;

    % --- (2) sigma_t0 범위 설정 ---
    sigma_t0_min = 0.001; 
    sigma_t0_max = deg2rad(80); % 필요하다면 더 늘려도 됨
    
    sigma_t0_vec = linspace(sigma_t0_min, sigma_t0_max, 100); 
    r_boundary = zeros(size(sigma_t0_vec));

    % ODE 함수
    ode_fun = @(sigma_t, s) s * tan((1 - s/2) * sigma_t);

    % --- (3) Loop Calculation ---
    for k = 1:length(sigma_t0_vec)
        curr_sigma_t0 = sigma_t0_vec(k);
        t_span = [curr_sigma_t0, 0];
        s0 = 1;
        
        [~, s_vec] = ode45(ode_fun, t_span, s0);
        s_final = s_vec(end);
        
        r_boundary(k) = r_allow / s_final;
    end

    % --- (4) 좌표 변환 ---
    plot_angle_vec = psi_t_rad + pi - sigma_t0_vec; 
    Region_X = Xt + r_boundary .* cos(plot_angle_vec);
    Region_Y = Yt + r_boundary .* sin(plot_angle_vec);

    % --- (5) 그리기 ---
    fill(ax, [Xt, Region_X, Xt], [Yt, Region_Y, Yt], 'y', ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
    plot(ax, Region_X, Region_Y, 'y--', 'LineWidth', 1.5, ...
        'DisplayName', 'Theory Boundary');
end