function Plot_Static_Results(cfg, sim_out)
    R2D = 180/pi;
    time = sim_out.time;
    hist_state = sim_out.hist_state;
    
    r_data = hist_state(1,:);
    lambda_data = hist_state(2,:) * R2D;
    psi_p_data = hist_state(3,:) * R2D; 
    psi_t_data = hist_state(4,:) * R2D;
    sigma_p_data = hist_state(5,:) * R2D;
    sigma_t_data = hist_state(6,:) * R2D;
    Vc_data = hist_state(7,:);
    acc_p_data = hist_state(8,:);

    % mode_flag 및 sigma_ref 데이터 추출 (배열 크기가 10 이상일 때만)
    has_extra_data = size(hist_state, 1) >= 10;
    if has_extra_data
        sigma_ref_data = hist_state(9,:) * R2D;
        mode_flag_data = hist_state(10,:);
    end

    % =========================================================
    % Figure 6
    % =========================================================
    fig6 = figure(6); set(fig6, 'Position', [10, 50, 500, 750], 'Theme', 'light');
    subplot(4,1,1); plot(time, r_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 6-1] Relative Distance');
    subplot(4,1,2); plot(time, -Vc_data, 'b-', 'LineWidth', 2); grid on; title('[Fig 6-2] Closing Velocity');
    subplot(4,1,3); plot(time, acc_p_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 6-3] Pursuer Acc');
    subplot(4,1,4); plot(r_data, acc_p_data, 'b-', 'LineWidth', 2); grid on; set(gca, 'XDir', 'reverse'); title('[Fig 6-4] Acc vs Range');

    % =========================================================
    % Figure 7
    % =========================================================
    fig7 = figure(7); set(fig7, 'Position', [520, 50, 500, 900], 'Theme', 'light');
    subplot(4,1,1); plot(time, lambda_data, 'm-', 'LineWidth', 2); grid on; title('[Fig 7-1] LOS Angle');
    
    % [수정] Lead Angles 플롯에 sigma_ref 추가
    subplot(4,1,2); hold on; grid on;
    plot(time, sigma_p_data, 'b-', 'LineWidth', 1.5, 'DisplayName', '\sigma_p');
    plot(time, sigma_t_data, 'r--', 'LineWidth', 1.5, 'DisplayName', '\sigma_t');
    if has_extra_data
        plot(time, sigma_ref_data, 'g-.', 'LineWidth', 1.5, 'DisplayName', '\sigma_{ref}');
    end
    title('[Fig 7-2] Lead Angles'); 
    legend('Location', 'best'); 
    hold off;

    subplot(4,1,3); plot(time, psi_p_data, 'b-', time, psi_t_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 7-3] Heading Angles');
    
    % [수정] Control Energy 대신 Guidance Mode Flag 플롯으로 교체
    subplot(4,1,4); hold on; grid on;
    if has_extra_data
        plot(time, mode_flag_data, 'k-', 'LineWidth', 2);
        yticks([-1 0 1 2]);
        ylim([-1.5, 2.5]);
        title('[Fig 7-4] Guidance Mode Flag (0:Safe, 1:2m, 2:r_{f,min})');
    else
        title('[Fig 7-4] Guidance Mode Flag (No Data)');
    end
    hold off;

    if strcmp(cfg.GUIDANCE_MODE, 'RDPG_MIN')
        fig8 = figure(8); set(fig8, 'Position', [1030, 50, 500, 160], 'Theme', 'light');
        plot(time, hist_state(17, :), 'r-', 'LineWidth', 2); grid on; 
        title('[Fig 8-1] r_{f} History');
    end

    % 이미지 저장 로직
    if cfg.auto_save == 1
        for idx = 1:8
            if ishandle(idx)
                filename = sprintf('Fig%02d_Result.png', idx);
                print(figure(idx), fullfile(cfg.save_dir, filename), '-dpng', '-r150'); 
            end
        end
    end
end