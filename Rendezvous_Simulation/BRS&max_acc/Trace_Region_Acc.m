function Trace_Region_Acc(sigma_p_deg, sigma_ref_deg)
    % 으헤~ DPG 도달 가능 영역이랑 가속도 포화 영역을 한 번에 그려주는 함수야.
    %
    % [입력 변수]
    % ax            : 그림을 그릴 축 (예: gca)
    % sigma_p_deg   : Pursuer의 리드각 (도 단위, deg)
    % r_f           : DPG 허용 반경 (Reachable Region 계산용)
    % sigma_ref_deg : RDPG 참조 각도 (도 단위, deg)
    % V_p           : Pursuer 속도
    % V_t           : Target 속도
    % gain_k        : 유도 게인 K
    % limit_acc     : 가속도 제한 (G)
    
    if nargin == 1
        sigma_ref_deg = sigma_p_deg;
    end
    
    r_f = 2;
    
    % 1. 첫 번째 함수 실행: DPG Reachable Region 그리기
    Trace_DPG_Region(sigma_p_deg, r_f);
    
    % 2. 두 번째 함수 실행: Max Acceleration Region 그리기
    Trace_Max_Acc(sigma_p_deg, sigma_ref_deg);
    
    % 3. 좌반 평면에 입력값 워드박스(텍스트 박스) 띄우기!
    ax = gca; % 현재 그려진 도화지를 가져와서~
    
    % 혹시 예전에 그려둔 박스가 있다면 지워주기 (글씨가 여러 번 겹치면 안 예쁘잖아~)
    delete(findobj(ax, 'Tag', 'AngleInfoBox'));
    
    % 박스에 적을 내용 만들기 (그리스 문자 sigma랑 도(deg) 기호도 넣었어!)
    box_text = sprintf('\\sigma_p = %.1f^\\circ\n\\sigma_{ref} = %.1f^\\circ', sigma_p_deg, sigma_ref_deg);
    
    % 텍스트 박스 도장 쾅!
    % (Units를 normalized로 하면 0.05, 0.95가 화면 왼쪽 위를 딱 가리키게 돼!)
    text(ax, 0.05, 0.95, box_text, ...
        'Units', 'normalized', ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [1 1 1 0.85], ... % 배경은 흰색에 살짝 투명하게!
        'EdgeColor', 'k', ...                % 테두리는 까만색!
        'Margin', 6, ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Tag', 'AngleInfoBox'); 
    
    % 으헤~ 다 그렸어. 참 쉽지?
end