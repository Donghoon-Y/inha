%% SM-2 MDATCOM Delta 출력파일 파싱 + 보간 + 2D 플롯
% 슬라이드 예시와 동일:
%   CM vs Alpha (delta별 색상)
%   CN vs Alpha (delta별 색상)
%   CMQ vs Alpha (Mach별 색상)
% 고도 5개 각각 별도 figure → 총 5개 figure (각 figure에 subplot 1x3)
clc; clear; close all;

%% ========== 설정 ==========
work_dir = '/Users/hoony/inha/ase4124/MDATCOM';
cd(work_dir);

output_files = {'cg1_delta_output.txt', 'cg2_delta_output.txt'};
cg_labels    = {'XCG=1m', 'XCG=2m'};

alpha_pts = [0, 5, 10, 15, 20];
mach_pts  = [0.5, 1.0, 1.5, 2.0, 2.5];
alt_pts   = [0, 2, 5, 10, 20];
delta_pts = [-30, -20, -10, 0, 10, 20, 30];

nA = length(alpha_pts);
nM = length(mach_pts);
nH = length(alt_pts);
nD = length(delta_pts);

%% ========== 파싱 함수 ==========
for cg_idx = 1:2
    fname = output_files{cg_idx};
    fprintf('파싱 중: %s\n', fname);

    CN  = nan(nA, nM, nH, nD);
    CM  = nan(nA, nM, nH, nD);
    CA  = nan(nA, nM, nH, nD);
    CMQ = nan(nA, nM, nH, nD);
    CNQ = nan(nA, nM, nH, nD);

    fid = fopen(fname, 'r');
    if fid < 0, error('파일 열기 실패: %s', fname); end
    lines = {};
    while ~feof(fid)
        ln = fgetl(fid);
        if ischar(ln), lines{end+1} = ln; end
    end
    fclose(fid);

    m_idx = -1; h_idx = -1; d_idx = -1;

    i = 1;
    while i <= length(lines)
        ln = lines{i};

        % CASEID에서 delta 인덱스
        if contains(ln, 'CASEID') && contains(ln, '_D')
            tok = regexp(ln, '_D(-?\d+)', 'tokens');
            if ~isempty(tok)
                d_val = str2double(tok{1}{1});
                [~, d_idx] = min(abs(delta_pts - d_val));
            end
            i = i+1; continue;
        end

        % 비행조건 헤더
        if contains(ln, 'MACH') && contains(ln, 'ALTITUDE') && contains(ln, 'VELOCITY')
            if i+3 <= length(lines)
                vals = str2num(strtrim(lines{i+3})); %#ok<ST2NM>
                if length(vals) >= 4
                    [~, m_idx] = min(abs(mach_pts - vals(1)));
                    [~, h_idx] = min(abs(alt_pts  - vals(2)/1000));
                end
            end
            i = i+1; continue;
        end

        % Dynamic: CNQ CMQ
        if contains(ln,'CNQ') && contains(ln,'CMQ') && m_idx>0 && h_idx>0 && d_idx>0
            row = i+1;
            for ai = 1:nA
                if row > length(lines), break; end
                dline = strtrim(lines{row});
                if isempty(dline); row=row+1; ai=ai-1; continue; end %#ok<FXSET>
                vals = str2num(dline); %#ok<ST2NM>
                if length(vals) >= 3
                    [~,ai_idx] = min(abs(alpha_pts - vals(1)));
                    CNQ(ai_idx,m_idx,h_idx,d_idx) = vals(2);
                    CMQ(ai_idx,m_idx,h_idx,d_idx) = vals(3);
                end
                row = row+1;
            end
            i = i+1; continue;
        end

        % Static: CNA CMA
        if contains(ln,'CNA') && contains(ln,'CMA') && ~contains(ln,'CNQ') ...
                && m_idx>0 && h_idx>0 && d_idx>0
            row = i+2;
            for ai = 1:nA
                if row > length(lines), break; end
                dline = strtrim(lines{row});
                if isempty(dline); row=row+1; ai=ai-1; continue; end %#ok<FXSET>
                vals = str2num(dline); %#ok<ST2NM>
                if length(vals) >= 9
                    [~,ai_idx] = min(abs(alpha_pts - vals(1)));
                    CN(ai_idx,m_idx,h_idx,d_idx) = vals(2);
                    CM(ai_idx,m_idx,h_idx,d_idx) = vals(3);
                    CA(ai_idx,m_idx,h_idx,d_idx) = vals(4);
                end
                row = row+1;
            end
            i = i+1; continue;
        end

        i = i+1;
    end

    fprintf('  NaN - CN:%d  CM:%d  CMQ:%d / 전체:%d\n', ...
        sum(isnan(CN(:))), sum(isnan(CM(:))), sum(isnan(CMQ(:))), nA*nM*nH*nD);

    %% ========== 보간 ==========
    alpha_fine = alpha_pts(1):1:alpha_pts(end);    % 0:1:20
    mach_fine  = mach_pts(1):0.05:mach_pts(end);  % 0.5:0.05:2.5
    delta_fine = delta_pts(1):1:delta_pts(end);    % -30:1:30

    [Aq,Mq,Hq,Dq] = ndgrid(alpha_fine, mach_fine, alt_pts, delta_fine);
    [A0,M0,H0,D0] = ndgrid(alpha_pts,  mach_pts,  alt_pts, delta_pts);

    CN_i  = interpn(A0,M0,H0,D0, CN,  Aq,Mq,Hq,Dq, 'spline');
    CM_i  = interpn(A0,M0,H0,D0, CM,  Aq,Mq,Hq,Dq, 'spline');
    try
        CMQ_i = interpn(A0,M0,H0,D0, CMQ, Aq,Mq,Hq,Dq, 'spline');
    catch
        CMQ_i = interpn(A0,M0,H0,D0, CMQ, Aq,Mq,Hq,Dq, 'linear');
    end

    d.CN_i=CN_i; d.CM_i=CM_i; d.CMQ_i=CMQ_i;
    d.alpha_fine=alpha_fine; d.mach_fine=mach_fine; d.delta_fine=delta_fine;

    if cg_idx==1, d1=d; else, d2=d; end
end

%% ========== 플롯 ==========
% CM, CN: delta별 색상 (7색)
% CMQ: Mach별 색상 (5색)

delta_plot  = [-30,-20,-10,0,10,20,30];
delta_labels = {'delta=-30','delta=-20','delta=-10','delta=0',...
                'delta=10','delta=20','delta=30'};
cmap_delta = [0.00 0.00 0.80;   % -30: 파랑
              0.00 0.55 1.00;   % -20: 하늘
              0.00 0.80 0.80;   % -10: 청록
              0.18 0.75 0.18;   %   0: 초록
              1.00 0.75 0.00;   %  10: 노랑
              1.00 0.35 0.00;   %  20: 주황
              0.70 0.00 0.70];  %  30: 보라

mach_plot  = [0.5, 1.0, 1.5, 2.0, 2.5];
mach_labels = {'M=0.50','M=1.00','M=1.50','M=2.00','M=2.50'};
cmap_mach  = [0.00 0.45 0.74;
              0.85 0.33 0.10;
              0.93 0.69 0.13;
              0.49 0.18 0.56;
              0.47 0.67 0.19];

% 대표 Mach (CMQ용 delta 고정 → delta=0)
[~, d0_idx]  = min(abs(d1.delta_fine - 0));

alt_labels_str = {'0m','2km','5km','10km','20km'};

for cg_idx = 1:2
    d = eval(sprintf('d%d', cg_idx));

    for hi = 1:nH
        figure('Name', sprintf('%s_ALT%s', cg_labels{cg_idx}, alt_labels_str{hi}), ...
               'Position', [50 50 1400 450]);
        sgtitle(sprintf('Delta/Alpha vs 공력계수  (%s, %s)', ...
            cg_labels{cg_idx}, alt_labels_str{hi}), ...
            'FontSize', 13, 'FontWeight', 'bold');

        %--- 서브플롯 1: CM vs Alpha (delta별) ---
        subplot(1,3,1); hold on;
        h_leg = gobjects(length(delta_plot),1);
        for di = 1:length(delta_plot)
            [~,di_idx] = min(abs(d.delta_fine - delta_plot(di)));
            % Mach 평균 (모든 Mach 중간값 사용: Mach=1.5)
            [~,mi_fix] = min(abs(d.mach_fine - 1.5));
            y = squeeze(d.CM_i(:, mi_fix, hi, di_idx));
            h_leg(di) = plot(d.alpha_fine, y, '-', ...
                'Color', cmap_delta(di,:), 'LineWidth', 1.5, ...
                'DisplayName', delta_labels{di});
        end
        yline(0,'k--','LineWidth',0.8);
        xlabel('Alpha (deg)'); ylabel('CM');
        title(sprintf('Delta -CM@ ALT %s -1', alt_labels_str{hi}));
        legend(h_leg, delta_labels, 'Location','best','FontSize',7);
        grid on;

        %--- 서브플롯 2: CN(=CZ) vs Alpha (delta별) ---
        subplot(1,3,2); hold on;
        h_leg2 = gobjects(length(delta_plot),1);
        for di = 1:length(delta_plot)
            [~,di_idx] = min(abs(d.delta_fine - delta_plot(di)));
            [~,mi_fix] = min(abs(d.mach_fine - 1.5));
            y = squeeze(d.CN_i(:, mi_fix, hi, di_idx));
            h_leg2(di) = plot(d.alpha_fine, y, '-', ...
                'Color', cmap_delta(di,:), 'LineWidth', 1.5, ...
                'DisplayName', delta_labels{di});
        end
        yline(0,'k--','LineWidth',0.8);
        xlabel('Alpha (deg)'); ylabel('CZ');
        title(sprintf('Delta -CZ@ ALT %s -1', alt_labels_str{hi}));
        legend(h_leg2, delta_labels, 'Location','best','FontSize',7);
        grid on;

        %--- 서브플롯 3: CMQ vs Alpha (Mach별, delta=0) ---
        subplot(1,3,3); hold on;
        h_leg3 = gobjects(length(mach_plot),1);
        for mi = 1:length(mach_plot)
            [~,mi_idx] = min(abs(d.mach_fine - mach_plot(mi)));
            y = squeeze(d.CMQ_i(:, mi_idx, hi, d0_idx));
            h_leg3(mi) = plot(d.alpha_fine, y, '-', ...
                'Color', cmap_mach(mi,:), 'LineWidth', 1.5, ...
                'DisplayName', mach_labels{mi});
        end
        yline(0,'k--','LineWidth',0.8);
        xlabel('Alpha (deg)'); ylabel('CMQ');
        title(sprintf('Alpha -CMQ@ ALT %s -1', alt_labels_str{hi}));
        legend(h_leg3, mach_labels, 'Location','best','FontSize',7);
        grid on;
    end
end

fprintf('\n=== 완료: CG1/CG2 각 5개 figure ===\n');