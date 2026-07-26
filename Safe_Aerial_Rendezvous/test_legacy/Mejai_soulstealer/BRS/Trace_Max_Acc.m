function Trace_Max_Acc(varargin)
    % Trace_Max_Acc : RDPG 유도 법칙 기반 가속도 포화 영역 시각화
    %
    % [사용법 1: 특정 축(ax)을 명시할 경우] (아저씨가 말한 4번째 입력부터 cfg 파라미터!)
    % Trace_Max_Acc(ax, sigma_p_deg, sigma_ref_deg, V_p, V_t, gain_k, limit_acc)
    %
    % [사용법 2: 자동으로 Figure 3에 그릴 경우]
    % Trace_Max_Acc(sigma_p_deg, sigma_ref_deg)
    
    %% 1. 기본 파라미터 설정 (main_rand.m 기준)
    def_V_p = 20.0;       % Pursuer 속도
    def_V_t = 20.0;       % Target 속도
    def_gain_k = 5.0;     % 유도 게인 K
    def_limit_acc = 1.0;  % 가속도 제한 (G)

    %% 2. 입력 변수 파싱 (스마트 처리)
    % 첫 번째 인자가 축(ax)이나 피규어 객체인지 확인
    if nargin > 0 && (isgraphics(varargin{1}, 'axes') || isgraphics(varargin{1}, 'figure'))
        if isgraphics(varargin{1}, 'figure')
            ax = gca(varargin{1});
        else
            ax = varargin{1};
        end
        idx = 1; % ax가 입력되었으므로 데이터 인덱스를 1만큼 밂
    else
        % ax가 없으면 자동으로 Figure 3 탐색 및 생성
        fig3 = findobj('Type', 'Figure', 'Name', '3. Target Centered with Saturation Map');
        if isempty(fig3)
            figure(3);
        else
            figure(fig3);
        end
        ax = gca;
        idx = 0;
    end
    
    % 필수 각도 파라미터 확인
    if nargin < idx + 2
        error('으헤... 최소한 sigma_p_deg와 sigma_ref_deg는 꼭 입력해 줘야 해~');
    end
    
    % 2~3번째(혹은 1~2번째) 입력: 각도
    sigma_p_deg   = varargin{idx + 1};
    sigma_ref_deg = varargin{idx + 2};
    
    % 4번째부터 선택적 파라미터 (입력 없으면 기본값 적용)
    if nargin >= idx + 3, V_p = varargin{idx + 3}; else, V_p = def_V_p; end
    if nargin >= idx + 4, V_t = varargin{idx + 4}; else, V_t = def_V_t; end
    if nargin >= idx + 5, K   = varargin{idx + 5}; else, K   = def_gain_k; end
    if nargin >= idx + 6, limit_acc = varargin{idx + 6}; else, limit_acc = def_limit_acc; end

    %% 3. 파라미터 추출 및 변환
    a_max = limit_acc * 9.81; % G -> m/s^2
    sigma_p = deg2rad(sigma_p_deg);
    sigma_ref = deg2rad(sigma_ref_deg);
    
    % Target 정보 (Target Centered Frame: (0,0), 90deg)
    Xt = 0; Yt = 0;
    psi_t_rad = pi/2; 

    %% 4. 경계 거리(r) 계산 (닫힌 다각형 꼼수 적용)
    sigma_t = linspace(0, 2*pi, 500); 
    
    % 공통 분자항 및 피드백 항
    numerator = V_p * (V_t * sin(sigma_t) - V_p * sin(sigma_p));
    term_feedback = V_p * K * (sigma_p - sigma_ref);
    
    % 가속도 한계별 도달 거리
    r_pos = numerator ./ (a_max + term_feedback);
    r_neg = numerator ./ (-a_max + term_feedback);
    
    % 음수 거리는 0으로 잘라서 영역 중심에 딱 붙이기
    r_pos_clamped = max(0, r_pos);
    r_neg_clamped = max(0, r_neg);
    
    %% 5. 좌표 변환 및 그리기
    plot_angle_vec = psi_t_rad + pi - sigma_t;
    
    X_pos = Xt + r_pos_clamped .* cos(plot_angle_vec);
    Y_pos = Yt + r_pos_clamped .* sin(plot_angle_vec);
    
    X_neg = Xt + r_neg_clamped .* cos(plot_angle_vec);
    Y_neg = Yt + r_neg_clamped .* sin(plot_angle_vec);
    
    hold(ax, 'on');
    
    % 애니메이션 효과를 위해 이전 영역 지우기
    delete(findobj(ax, 'Tag', 'PosAccFill'));
    delete(findobj(ax, 'Tag', 'PosAccLine'));
    delete(findobj(ax, 'Tag', 'NegAccFill'));
    delete(findobj(ax, 'Tag', 'NegAccLine'));
    
    % --- [+Acc Limit] 영역 (Red) ---
    if any(r_pos > 0)
        % 'c'를 'r'로 변경 (빨간색 영역)
        fill(ax, X_pos, Y_pos, 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
            'Tag', 'PosAccFill', 'HandleVisibility', 'off');
        % 'c-.'를 'r:'로 변경 (빨간색 점선)
        plot(ax, X_pos, Y_pos, 'r:', 'LineWidth', 1.5, ...
            'DisplayName', '+Acc Limit', 'Tag', 'PosAccLine');
    end
    
    % --- [-Acc Limit] 영역 (Blue) ---
    if any(r_neg > 0)
        % 'm'을 'b'로 변경 (초록색 영역)
        fill(ax, X_neg, Y_neg, 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
            'Tag', 'NegAccFill', 'HandleVisibility', 'off');
        % 'm-.'를 'b:'로 변경 (초록색 점선)
        plot(ax, X_neg, Y_neg, 'g:', 'LineWidth', 1.5, ...
            'DisplayName', '-Acc Limit', 'Tag', 'NegAccLine');
    end
end