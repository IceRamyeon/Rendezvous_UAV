% Plot of f(sigma_p; sigma_t) and C for different sigma_p values, with max point and intersection points highlighted.
% This code is a part of the analysis for Theorem 1 verification in the context of maximum acceleration requirements in a rendezvous simulation.
clear; clc; close all;

%% 1. Inputs & Parameters
r_f = 2;              % [m]
a_max_input = 1;      % [g]
sigma_p_deg_array = 30;  % desired sigma_pc values in degrees
epsilon = 1e-7;         % Prevent division by zero in case cos(sigma_p) is very small
V = 20;               % [m/s] 
n = 50000;            % # of points
ub = pi;              % upper bound for sigma_t (0 to pi)
ub_deg = rad2deg(ub); % upper bound in degrees

% 단위 변환 및 계산
a_max = a_max_input * 9.81;       % [m/s^2]
sigma_t = linspace(0, ub, n);     

%% 2. Function Calculation & 3. Plots
% one figure per sigma_p_deg value, with 3 subplots for each case
figure('Name', 'Guidance Reachable Set Analysis', 'Theme', 'light');

% repeat the analysis for each sigma_p_deg value
for k = 1:length(sigma_p_deg_array)
    sigma_p_deg = sigma_p_deg_array(k);
    sigma_p = deg2rad(sigma_p_deg);   % [rad]
    
    % Calculate constant C for the current sigma_p
    C = (2 * r_f * a_max) / (V^2);
    
    % Calculate the left-hand side function f(sigma_t) for the current sigma_p
    y = (sin(sigma_t) - sin(sigma_p)) .* (1 + cos(sigma_t + sigma_p)) / (cos(sigma_p)^2 + epsilon);
    
    % 3x1 subplot for the current sigma_p case
    subplot(1, 1, k);
    hold on; grid on;
    
    % Left-hand side graph
    plot(rad2deg(sigma_t), y, 'b-', 'LineWidth', 2, 'DisplayName', 'LHS: f(\sigma_t)');
    
    % Right-hand side constant line
    yline(C, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('RHS: C = %.4f', C));
    
    % Calculate and plot the maximum r_f of the function f(sigma_p; sigma_t)
    [y_max, max_idx] = max(y);  % Maximum value of the function and its index
    sigma_t_max = sigma_t(max_idx);
    
    % Calculate the corresponding r_f at the maximum point using the formula: r_f = (y_max * V^2) / (2 * a_max)
    r_f_max = (y_max * V^2) / (2 * a_max);
    
    % Mark the maximum point on the graph with a red marker
    plot(rad2deg(sigma_t_max), y_max, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    
    % plot the text for the maximum point with both angle and r_f value
    txt_max = sprintf(' Max: %.1f\\circ\n r_{f,max} = %.2f m', rad2deg(sigma_t_max), r_f_max);
    text(rad2deg(sigma_t_max), y_max + 0.1, txt_max, 'FontSize', 15, 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    
    % Find and plot the intersection points where f(sigma_t) crosses C
    diff_y = y - C;
    cross_idx = find(diff_y(1:end-1) .* diff_y(2:end) <= 0);
    
    for i = 1:length(cross_idx)
        idx = cross_idx(i);
        x_cross = rad2deg(sigma_t(idx)); % x value of the intersection point in degrees
        
        % find the corresponding r_f value at the intersection point using the formula: r_f = (y(idx) * V^2) / (2 * a_max)
        r_val = (V^2 / a_max) * (sin(sigma_t(idx)) - sin(sigma_p));
        
        % Plot the intersection point with a yellow marker
        plot(x_cross, C, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
        
        % Text for the intersection point showing the angle and corresponding r_f value
        txt = sprintf(' %.1f\\circ\n r = %.2f m', x_cross, r_val);
        text(x_cross, C - 0.3, txt, 'FontSize', 15, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
    end
    
    % Plot Graph Settings
    xlabel('\sigma_t [degree]', 'FontSize', 15, 'FontWeight', 'bold');
    ylabel('Value of C', 'FontSize', 15, 'FontWeight', 'bold');
    title(sprintf('\\sigma_{pc} = %.1f\\circ', sigma_p_deg), 'FontSize', 20);
    xlim([0 ub_deg]);
    ylim([min(-0.2, min(y)-0.1) max(max(y)+0.5, C-0.05)]); 
    
    legend('Location', 'best', 'FontSize', 20);    
    hold off;
end