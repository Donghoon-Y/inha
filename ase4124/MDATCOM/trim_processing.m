%% SM-2 MDATCOM TRIM 출력 파싱 + DELTA vs Alpha 플롯
% TRIM 출력에서 각 alpha에서의 트림 조종면 변위각(DELTA) 추출
% NT(No Trim) → NaN 처리
clc; clear; close all;

%% ========== 설정 ==========
work_dir = '/Users/hoony/inha/ase4124/MDATCOM';
cd(work_dir);

output_files = {'cg1_trim_output.txt', 'cg2_trim_output.txt'};
cg_labels    = {'XCG=1m', 'XCG=2m'};

alpha_pts = [0, 5, 10, 15, 20];
mach_pts  = [0.5, 1.01, 1.5, 2.0, 2.5];
alt_pts   = [0, 2, 5, 10, 20];
delta_pts = [-30, -20, -10, 0, 10, 20, 30];

nA = length(alpha_pts);
nM = length(mach_pts);
nH = length(alt_pts);
nD = length(delta_pts);

%% ========== 파싱 ==========
% DELTA_trim(alpha, Mach, 고도, input_delta)
% input_delta: 입력한 delta값 (각 케이스)
% DELTA_trim: 그 조건에서 트림된 실제 조종면 각도

all_data = cell(1,2);

for cg_idx = 1:2
    fname = output_files{cg_idx};
    fprintf('파싱 중: %s\n', fname);

    % [alpha x Mach x 고도 x input_delta]
    DELTA_trim = nan(nA, nM, nH, nD);
    CN_trim    = nan(nA, nM, nH, nD);
    CA_trim    = nan(nA, nM, nH, nD);

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

        % CASEID에서 input delta 인덱스
        if contains(ln,'CASEID') && contains(ln,'_D')
            tok = regexp(ln, '_D(-?\d+)', 'tokens');
            if ~isempty(tok)
                d_val = str2double(tok{1}{1});
                [~,d_idx] = min(abs(delta_pts - d_val));
            end
            i = i+1; continue;
        end

        % 비행조건 헤더에서 Mach, 고도
        if contains(ln,'MACH') && contains(ln,'ALTITUDE') && contains(ln,'VELOCITY')
            if i+3 <= length(lines)
                vals = str2num(strtrim(lines{i+3})); %#ok<ST2NM>
                if length(vals) >= 4
                    [~,m_idx] = min(abs(mach_pts - vals(1)));
                    [~,h_idx] = min(abs(alt_pts  - vals(2)/1000));
                end
            end
            i = i+1; continue;
        end

        % TRIMMED STATIC 데이터 블록
        % 헤더: "ALPHA    DELTA    CL    CD    CN    CA ..."
        if contains(ln,'ALPHA') && contains(ln,'DELTA') && contains(ln,'CN') ...
                && contains(ln,'CA') && m_idx>0 && h_idx>0 && d_idx>0
            % 빈 줄 건너뛰고 데이터 읽기
            row = i+1;
            ai_count = 0;
            while ai_count < nA && row <= length(lines)
                dline = strtrim(lines{row});
                row = row+1;
                if isempty(dline), continue; end
                % NT 포함 여부 확인
                if contains(dline, '*NT*')
                    % NT인 줄에서 alpha값만 추출
                    parts = strsplit(dline);
                    alpha_val = str2double(parts{1});
                    if ~isnan(alpha_val)
                        [~,ai_idx] = min(abs(alpha_pts - alpha_val));
                        % DELTA_trim, CN, CA 모두 NaN 유지
                        ai_count = ai_count+1;
                    end
                else
                    vals = str2num(dline); %#ok<ST2NM>
                    if length(vals) >= 6
                        alpha_val = vals(1);
                        [~,ai_idx] = min(abs(alpha_pts - alpha_val));
                        DELTA_trim(ai_idx, m_idx, h_idx, d_idx) = vals(2); % 트림 delta
                        CN_trim   (ai_idx, m_idx, h_idx, d_idx) = vals(5); % CN
                        CA_trim   (ai_idx, m_idx, h_idx, d_idx) = vals(6); % CA
                        ai_count = ai_count+1;
                    end
                end
            end
            i = i+1; continue;
        end

        i = i+1;
    end

    nan_cnt = sum(isnan(DELTA_trim(:)));
    tot_cnt = numel(DELTA_trim);
    fprintf('  DELTA_trim NaN: %d / %d (NT 포함)\n', nan_cnt, tot_cnt);

    d.DELTA_trim = DELTA_trim;
    d.CN_trim    = CN_trim;
    d.CA_trim    = CA_trim;
    all_data{cg_idx} = d;
end

%% ========== 플롯: DELTA vs Alpha (Mach별, 고도별, input_delta별) ==========
% 각 고도에서: subplot(1,2) → CG1 / CG2
% x축: Alpha, y축: 트림 DELTA
% 색상: input_delta별 (7색)
% 선 스타일: Mach별

cmap_delta = [0.00 0.00 0.80;
              0.00 0.55 1.00;
              0.00 0.80 0.80;
              0.18 0.75 0.18;
              1.00 0.75 0.00;
              1.00 0.35 0.00;
              0.70 0.00 0.70];
delta_labels = {'\delta_{in}=-30°','\delta_{in}=-20°','\delta_{in}=-10°',...
                '\delta_{in}=0°','\delta_{in}=10°','\delta_{in}=20°','\delta_{in}=30°'};

cmap_mach = [0.00 0.45 0.74;
             0.85 0.33 0.10;
             0.93 0.69 0.13;
             0.49 0.18 0.56;
             0.47 0.67 0.19];
mach_labels = {'M=0.50','M=1.01','M=1.50','M=2.00','M=2.50'};
line_styles = {'-','--',':','-.', '-'};
alt_labels  = {'0km','2km','5km','10km','20km'};

% 플롯 1: 고도별 figure, Mach별 색상, input_delta별 선 스타일
for hi = 1:nH
    figure('Name', sprintf('DELTA_trim_ALT%s', alt_labels{hi}), ...
           'Position', [50 50 1100 480]);
    sgtitle(sprintf('트림 조종면 변위각 \\delta_{trim} vs \\alpha  (%s)', alt_labels{hi}), ...
        'FontSize', 13, 'FontWeight', 'bold');

    for cg_idx = 1:2
        d = all_data{cg_idx};
        subplot(1,2,cg_idx); hold on;

        % 대표: input_delta=0일 때 Mach별로 그리기
        [~,d0] = min(abs(delta_pts - 0));
        h_leg = gobjects(nM,1);
        for mi = 1:nM
            y = squeeze(d.DELTA_trim(:, mi, hi, d0));
            h_leg(mi) = plot(alpha_pts, y, 'o-', ...
                'Color', cmap_mach(mi,:), 'LineWidth', 1.5, ...
                'DisplayName', mach_labels{mi});
        end

        yline(0,'k--','LineWidth',0.8);
        xlabel('\alpha [deg]'); ylabel('\delta_{trim} [deg]');
        title(sprintf('%s', cg_labels{cg_idx}));
        legend(h_leg, mach_labels, 'Location','best','FontSize',8);
        grid on;
    end
end

% 플롯 2: input_delta별로 DELTA_trim 비교 (고도=0km, CG1 vs CG2)
figure('Name','DELTA_trim_vs_inputdelta','Position',[50 50 1100 480]);
sgtitle('트림 \delta_{trim} vs \alpha  (고도=0km, input \delta별)', ...
    'FontSize',13,'FontWeight','bold');

hi_fix = 1;  % 고도=0km
[~,mi_fix] = min(abs(mach_pts - 1.5));  % Mach=1.5

for cg_idx = 1:2
    d = all_data{cg_idx};
    subplot(1,2,cg_idx); hold on;

    h_leg = gobjects(nD,1);
    for di = 1:nD
        y = squeeze(d.DELTA_trim(:, mi_fix, hi_fix, di));
        h_leg(di) = plot(alpha_pts, y, 'o-', ...
            'Color', cmap_delta(di,:), 'LineWidth', 1.5, ...
            'DisplayName', delta_labels{di});
    end

    yline(0,'k--','LineWidth',0.8);
    xlabel('\alpha [deg]'); ylabel('\delta_{trim} [deg]');
    title(sprintf('%s  (M=1.5, %s)', cg_labels{cg_idx}, alt_labels{hi_fix}));
    legend(h_leg, delta_labels, 'Location','best','FontSize',7);
    grid on;
end

% 플롯 3: CN_trim vs Alpha (트림된 CN, Mach별, 고도별)
figure('Name','CN_trim','Position',[50 50 1100 480]);
sgtitle('트림된 CN vs \alpha  (고도=0km)', 'FontSize',13,'FontWeight','bold');

hi_fix = 1;
[~,d0] = min(abs(delta_pts - 0));

for cg_idx = 1:2
    d = all_data{cg_idx};
    subplot(1,2,cg_idx); hold on;

    h_leg = gobjects(nM,1);
    for mi = 1:nM
        y = squeeze(d.CN_trim(:, mi, hi_fix, d0));
        h_leg(mi) = plot(alpha_pts, y, 'o-', ...
            'Color', cmap_mach(mi,:), 'LineWidth', 1.5, ...
            'DisplayName', mach_labels{mi});
    end

    xlabel('\alpha [deg]'); ylabel('CN_{trim}');
    title(sprintf('%s', cg_labels{cg_idx}));
    legend(h_leg, mach_labels, 'Location','best','FontSize',8);
    grid on;
end

fprintf('\n=== 완료 ===\n');