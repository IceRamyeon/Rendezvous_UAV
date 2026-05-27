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
    subplot(4,1,2); plot(time, sigma_p_data, 'b-', time, sigma_t_data, 'r--', 'LineWidth', 1.5); grid on; title('[Fig 7-2] Lead Angles');
    subplot(4,1,3); plot(time, psi_p_data, 'b-', time, psi_t_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 7-3] Heading Angles');
    
    energy_p = cumtrapz(time, acc_p_data.^2);
    subplot(4,1,4); plot(time, energy_p, 'c-', 'LineWidth', 2); grid on; title('[Fig 7-4] Control Energy');

    % =========================================================
    % Figure 8 (RDPG_SAFE 전용)
    % =========================================================
    if strcmp(cfg.GUIDANCE_MODE, 'RDPG_SAFE')
        fig8 = figure(8); set(fig8, 'Position', [1030, 50, 500, 750], 'Theme', 'light');
        subplot(3, 1, 1); plot(time, hist_state(10, :), 'b-', 'LineWidth', 2); grid on; title('CLF');
        subplot(3, 1, 2); plot(time, hist_state(11, :), 'r-', 'LineWidth', 2); grid on; title('CBF Margin');
        subplot(3, 1, 3); plot(time, hist_state(12, :), 'm-', 'LineWidth', 2); grid on; title('Slack Variable');
    end

    % 이미지 저장 로직
    if cfg.auto_save == 1
        fprintf('>>> 그래프 이미지 저장 시작...\n');
        for idx = 1:8
            if ishandle(idx)
                filename = sprintf('Fig%02d_Result.png', idx);
                print(figure(idx), fullfile(cfg.save_dir, filename), '-dpng', '-r150'); 
            end
        end
        fprintf('>>> 이미지 저장 완료!\n');
    end
end