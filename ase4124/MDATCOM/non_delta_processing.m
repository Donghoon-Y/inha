%% SM-2 MDATCOM 출력파일 파싱 + 보간 + 플롯
% 무차원: CN, CM, CA, CNA, CMA, CMQ (고도별, CG1 vs CG2)
% 유차원 안정성 미분계수: Za, Zd, Ma, Md, Lp (고도별, 설계점별)
clc; clear; close all;

%% ========== 설정 ==========
work_dir = '/Users/hoony/inha/ase4124/MDATCOM';
cd(work_dir);

output_files      = {'cg1_output.txt',       'cg2_output.txt'};
delta_output_files= {'cg1_delta_output.txt', 'cg2_delta_output.txt'};

alpha_pts = [0, 5, 10, 15, 20];
mach_pts  = [0.5, 1.01, 1.5, 2.0, 2.5];
alt_pts   = [0, 2, 5, 10, 20];
delta_pts = [-30, -20, -10, 0, 10, 20, 30];

nA = length(alpha_pts);
nM = length(mach_pts);
nH = length(alt_pts);
nD = length(delta_pts);

% 참조값
SREF = 0.092;   % [m^2]
LREF = 0.343;   % [m]

% CATIA → MDATCOM 축 변환
% CATIA Y축 = 기체 종축 → MDATCOM X축
% CATIA Ixx(G) = MDATCOM Iyy (피치 관성모멘트)
% CATIA Iyy(G) = MDATCOM Ixx (롤 관성모멘트)
mass_cg   = [707.618,  707.493];   % [kg]
Iyy_pitch = [1063.369, 1645.475];  % [kg·m²] 피치 (CATIA Ixx → MDATCOM Iyy)
Ixx_roll  = [12.239,   14.216];    % [kg·m²] 롤   (CATIA Iyy → MDATCOM Ixx)

%% ========== 파싱 함수 ==========
function d = parse_datcom(fname, alpha_pts, mach_pts, alt_pts)
    nA = length(alpha_pts);
    nM = length(mach_pts);
    nH = length(alt_pts);

    CN  = nan(nA,nM,nH); CM  = nan(nA,nM,nH);
    CA  = nan(nA,nM,nH); CNA = nan(nA,nM,nH);
    CMA = nan(nA,nM,nH); CMQ = nan(nA,nM,nH);
    CNQ = nan(nA,nM,nH);
    q_dyn = nan(nM,nH);  V_inf = nan(nM,nH);

    fid = fopen(fname,'r');
    if fid < 0, error('파일 열기 실패: %s', fname); end
    lines = {};
    while ~feof(fid)
        ln = fgetl(fid);
        if ischar(ln), lines{end+1} = ln; end
    end
    fclose(fid);

    m_idx=-1; h_idx=-1;
    i=1;
    while i <= length(lines)
        ln = lines{i};

        if contains(ln,'MACH') && contains(ln,'ALTITUDE') && contains(ln,'VELOCITY')
            if i+3 <= length(lines)
                vals = str2num(strtrim(lines{i+3})); %#ok<ST2NM>
                if length(vals) >= 4
                    [~,m_idx] = min(abs(mach_pts - vals(1)));
                    [~,h_idx] = min(abs(alt_pts  - vals(2)/1000));
                    q_dyn(m_idx,h_idx) = vals(4);
                    V_inf(m_idx,h_idx) = vals(3);
                end
            end
            i=i+1; continue;
        end

        if contains(ln,'CNQ') && contains(ln,'CMQ') && m_idx>0 && h_idx>0
            row=i+1;
            for ai=1:nA
                if row>length(lines), break; end
                dline=strtrim(lines{row});
                if isempty(dline); row=row+1; ai=ai-1; continue; end %#ok<FXSET>
                vals=str2num(dline); %#ok<ST2NM>
                if length(vals)>=3
                    [~,ai_idx]=min(abs(alpha_pts-vals(1)));
                    CNQ(ai_idx,m_idx,h_idx)=vals(2);
                    CMQ(ai_idx,m_idx,h_idx)=vals(3);
                end
                row=row+1;
            end
            i=i+1; continue;
        end

        if contains(ln,'CNA') && contains(ln,'CMA') && ~contains(ln,'CNQ') ...
                && m_idx>0 && h_idx>0
            row=i+2;
            for ai=1:nA
                if row>length(lines), break; end
                dline=strtrim(lines{row});
                if isempty(dline); row=row+1; ai=ai-1; continue; end %#ok<FXSET>
                vals=str2num(dline); %#ok<ST2NM>
                if length(vals)>=9
                    [~,ai_idx]=min(abs(alpha_pts-vals(1)));
                    CN(ai_idx,m_idx,h_idx)=vals(2);
                    CM(ai_idx,m_idx,h_idx)=vals(3);
                    CA(ai_idx,m_idx,h_idx)=vals(4);
                    CNA(ai_idx,m_idx,h_idx)=vals(8);
                    CMA(ai_idx,m_idx,h_idx)=vals(9);
                end
                row=row+1;
            end
            i=i+1; continue;
        end
        i=i+1;
    end

    % 보간
    alpha_fine = alpha_pts(1):1:alpha_pts(end);
    mach_fine  = mach_pts(1):0.05:mach_pts(end);
    alt_fine   = alt_pts(1):1:alt_pts(end);

    [Aq,Mq,Hq] = ndgrid(alpha_fine,mach_fine,alt_fine);
    [A0,M0,H0] = ndgrid(alpha_pts, mach_pts, alt_pts);

    CN_i  = interpn(A0,M0,H0,CN, Aq,Mq,Hq,'spline');
    CM_i  = interpn(A0,M0,H0,CM, Aq,Mq,Hq,'spline');
    CA_i  = interpn(A0,M0,H0,CA, Aq,Mq,Hq,'spline');
    CNA_i = interpn(A0,M0,H0,CNA,Aq,Mq,Hq,'spline');
    CMA_i = interpn(A0,M0,H0,CMA,Aq,Mq,Hq,'spline');
    try
        CMQ_i = interpn(A0,M0,H0,CMQ,Aq,Mq,Hq,'spline');
    catch
        CMQ_i = interpn(A0,M0,H0,CMQ,Aq,Mq,Hq,'linear');
    end

    d.CN=CN; d.CM=CM; d.CA=CA; d.CNA=CNA; d.CMA=CMA; d.CMQ=CMQ;
    d.CN_i=CN_i; d.CM_i=CM_i; d.CA_i=CA_i;
    d.CNA_i=CNA_i; d.CMA_i=CMA_i; d.CMQ_i=CMQ_i;
    d.q_dyn=q_dyn; d.V_inf=V_inf;
    d.alpha_fine=alpha_fine; d.mach_fine=mach_fine; d.alt_fine=alt_fine;
end

function d = parse_datcom_delta(fname, alpha_pts, mach_pts, alt_pts, delta_pts)
    nA=length(alpha_pts); nM=length(mach_pts);
    nH=length(alt_pts);   nD=length(delta_pts);

    CN = nan(nA,nM,nH,nD);
    CM = nan(nA,nM,nH,nD);

    fid=fopen(fname,'r');
    if fid<0, error('파일 열기 실패: %s',fname); end
    lines={};
    while ~feof(fid)
        ln=fgetl(fid);
        if ischar(ln), lines{end+1}=ln; end
    end
    fclose(fid);

    m_idx=-1; h_idx=-1; d_idx=-1;
    i=1;
    while i<=length(lines)
        ln=lines{i};
        if contains(ln,'CASEID') && contains(ln,'_D')
            tok=regexp(ln,'_D(-?\d+)','tokens');
            if ~isempty(tok)
                [~,d_idx]=min(abs(delta_pts-str2double(tok{1}{1})));
            end
            i=i+1; continue;
        end
        if contains(ln,'MACH') && contains(ln,'ALTITUDE') && contains(ln,'VELOCITY')
            if i+3<=length(lines)
                vals=str2num(strtrim(lines{i+3})); %#ok<ST2NM>
                if length(vals)>=4
                    [~,m_idx]=min(abs(mach_pts-vals(1)));
                    [~,h_idx]=min(abs(alt_pts-vals(2)/1000));
                end
            end
            i=i+1; continue;
        end
        if contains(ln,'CNA') && contains(ln,'CMA') && ~contains(ln,'CNQ') ...
                && m_idx>0 && h_idx>0 && d_idx>0
            row=i+2;
            for ai=1:nA
                if row>length(lines), break; end
                dline=strtrim(lines{row});
                if isempty(dline); row=row+1; ai=ai-1; continue; end %#ok<FXSET>
                vals=str2num(dline); %#ok<ST2NM>
                if length(vals)>=9
                    [~,ai_idx]=min(abs(alpha_pts-vals(1)));
                    CN(ai_idx,m_idx,h_idx,d_idx)=vals(2);
                    CM(ai_idx,m_idx,h_idx,d_idx)=vals(3);
                end
                row=row+1;
            end
            i=i+1; continue;
        end
        i=i+1;
    end
    d.CN=CN; d.CM=CM;
end

%% ========== 데이터 로드 ==========
fprintf('=== 기본 출력 파싱 ===\n');
all_data = cell(1,2);
for cg_idx=1:2
    fprintf('파싱: %s\n', output_files{cg_idx});
    all_data{cg_idx} = parse_datcom(output_files{cg_idx}, alpha_pts, mach_pts, alt_pts);
end

fprintf('\n=== Delta 출력 파싱 ===\n');
all_delta = cell(1,2);
for cg_idx=1:2
    fprintf('파싱: %s\n', delta_output_files{cg_idx});
    all_delta{cg_idx} = parse_datcom_delta(delta_output_files{cg_idx}, ...
        alpha_pts, mach_pts, alt_pts, delta_pts);
end

d1=all_data{1}; d2=all_data{2};

%% ========== 색상 ==========
colors_mach = [0.00 0.45 0.74;
               0.85 0.33 0.10;
               0.93 0.69 0.13;
               0.49 0.18 0.56;
               0.47 0.67 0.19];
mach_labels = {'M=0.50','M=1.01','M=1.50','M=2.00','M=2.50'};
alt_labels  = {'0km','2km','5km','10km','20km'};
colors_alt  = jet(nH);

%% ========== 플롯 1: 무차원 공력계수 6종 ==========
coeff_list  = {'CN_i','CM_i','CA_i','CNA_i','CMA_i','CMQ_i'};
coeff_names = {'CN','CM','CA','CNA','CMA','CMQ'};
ylabels     = {'CN','CM','CA','CNA [/deg]','CMA [/deg]','CMQ [/deg]'};

for ci=1:6
    figure('Name',coeff_names{ci},'Position',[50 50 1000 900]);
    sgtitle(sprintf('%s vs \\alpha  (고도별, CG1 vs CG2)',coeff_names{ci}),...
        'FontSize',13,'FontWeight','bold');
    for hi=1:nH
        for cg_idx=1:2
            d=all_data{cg_idx};
            subplot(nH,2,(hi-1)*2+cg_idx); hold on;
            for mi=1:nM
                [~,mi_f]=min(abs(d.mach_fine-mach_pts(mi)));
                [~,hi_f]=min(abs(d.alt_fine-alt_pts(hi)));
                y=squeeze(d.(coeff_list{ci})(:,mi_f,hi_f));
                plot(d.alpha_fine,y,'-','Color',colors_mach(mi,:),...
                    'LineWidth',1.5,'DisplayName',mach_labels{mi});
            end
            if hi==1, legend('Location','best','FontSize',7); end
            yline(0,'k--','LineWidth',0.8);
            xlabel('\alpha [deg]'); ylabel(ylabels{ci});
            title(sprintf('XCG=%dm, %s',cg_idx,alt_labels{hi})); grid on;
        end
    end
end

%% ========== 플롯 2: 세로 안정성 비교 CG1 vs CG2 ==========
figure('Name','Stability','Position',[50 50 1200 900]);
sgtitle('세로 안정성 비교: XCG=1m vs XCG=2m','FontSize',13,'FontWeight','bold');
mach_fix=1.5;
[~,mi_fix]=min(abs(d1.mach_fine-mach_fix));

titles2 = {'CM vs \alpha','CMA vs \alpha','CMQ vs \alpha','CMA vs Mach','CMQ vs Mach'};
fields1 = {'CM_i','CMA_i','CMQ_i','CMA_i','CMQ_i'};
ylabs2  = {'CM','CMA [/deg]','CMQ [/deg]','CMA [/deg]','CMQ [/deg]'};

for pi=1:5
    subplot(2,3,pi); hold on;
    for hi=1:nH
        [~,hi_f]=min(abs(d1.alt_fine-alt_pts(hi)));
        if pi<=3  % vs alpha
            plot(d1.alpha_fine,squeeze(d1.(fields1{pi})(:,mi_fix,hi_f)),'-',...
                'Color',colors_alt(hi,:),'LineWidth',1.5,'DisplayName',['CG1 ',alt_labels{hi}]);
            plot(d2.alpha_fine,squeeze(d2.(fields1{pi})(:,mi_fix,hi_f)),'--',...
                'Color',colors_alt(hi,:),'LineWidth',1.5,'DisplayName',['CG2 ',alt_labels{hi}]);
            xlabel('\alpha [deg]');
        else  % vs Mach
            plot(d1.mach_fine,squeeze(d1.(fields1{pi})(1,:,hi_f)),'-',...
                'Color',colors_alt(hi,:),'LineWidth',1.5,'DisplayName',['CG1 ',alt_labels{hi}]);
            plot(d2.mach_fine,squeeze(d2.(fields1{pi})(1,:,hi_f)),'--',...
                'Color',colors_alt(hi,:),'LineWidth',1.5,'DisplayName',['CG2 ',alt_labels{hi}]);
            xlabel('Mach');
        end
    end
    yline(0,'k--'); ylabel(ylabs2{pi}); title(titles2{pi});
    legend('FontSize',5,'Location','best'); grid on;
end
subplot(2,3,6); axis off;
text(0.1,0.7,'실선: XCG=1m','FontSize',12);
text(0.1,0.5,'점선: XCG=2m','FontSize',12);
text(0.1,0.3,'색상: 고도별','FontSize',12);

%% ========== 플롯 3: 유차원 안정성 미분계수 ==========
% Za = CNA × q × S / m              [m/s² / rad]
% Ma = CMA × q × S × L / Iyy        [1/s² / rad]
% Lp = CMQ × q × S × L² / (2V×Iyy) [1/s]
% Zd = (CN(d+)-CN(d-))/(2Δd) × q×S/m          수치미분
% Md = (CM(d+)-CM(d-))/(2Δd) × q×S×L/Iyy      수치미분

fprintf('\n=== 유차원 안정성 미분계수 계산 ===\n');

% delta 수치미분: d+10 vs d-10 (중앙차분)
[~,di_p]=min(abs(delta_pts-10));   % delta=+10
[~,di_m]=min(abs(delta_pts-(-10)));% delta=-10
dd = (10-(-10)) * pi/180;           % [rad]

for cg_idx=1:2
    d   = all_data{cg_idx};
    dd_data = all_delta{cg_idx};
    m   = mass_cg(cg_idx);
    Iyy = Iyy_pitch(cg_idx);

    figure('Name',sprintf('StabilityDerivatives_CG%d',cg_idx),...
           'Position',[50 50 1300 900]);
    sgtitle(sprintf('유차원 안정성 미분계수  (XCG=%dm,  m=%.1fkg,  Iyy=%.1fkg·m²)',...
        cg_idx, m, Iyy),'FontSize',12,'FontWeight','bold');

    deriv_titles = {'Z_\alpha [m/s²/rad]','M_\alpha [1/s²/rad]',...
                    'Z_\delta [m/s²/rad]','M_\delta [1/s²/rad]','L_p [1/s]'};

    for hi=1:nH
        for pi=1:5
            subplot(nH,5,(hi-1)*5+pi); hold on;

            for mi=1:nM
                q = d.q_dyn(mi,hi);
                V = d.V_inf(mi,hi);
                if isnan(q)||isnan(V), continue; end

                % 무차원 계수 (alpha=0 기준 단일값)
                CNA_val = d.CNA(1,mi,hi);  % /deg → /rad 변환 필요
                CMA_val = d.CMA(1,mi,hi);
                CMQ_val = d.CMQ(1,mi,hi);

                % deg→rad 변환 (DERIV DEG 사용했으므로)
                CNA_rad = CNA_val * (180/pi);
                CMA_rad = CMA_val * (180/pi);
                CMQ_rad = CMQ_val * (180/pi);

                % delta 수치미분 (alpha별로)
                CN_p = dd_data.CN(:,mi,hi,di_p);
                CN_m = dd_data.CN(:,mi,hi,di_m);
                CM_p = dd_data.CM(:,mi,hi,di_p);
                CM_m = dd_data.CM(:,mi,hi,di_m);
                CNd  = (CN_p - CN_m) / dd;  % /rad [nA x 1]
                CMd  = (CM_p - CM_m) / dd;

                if pi==1
                    % Za = CNA/rad × q × S / m
                    y = CNA_rad * q * SREF / m;
                    plot(alpha_pts, y*ones(nA,1), '-', ...
                        'Color',colors_mach(mi,:),'LineWidth',1.5,...
                        'DisplayName',mach_labels{mi});
                elseif pi==2
                    % Ma = CMA/rad × q × S × L / Iyy
                    y = CMA_rad * q * SREF * LREF / Iyy;
                    plot(alpha_pts, y*ones(nA,1), '-', ...
                        'Color',colors_mach(mi,:),'LineWidth',1.5,...
                        'DisplayName',mach_labels{mi});
                elseif pi==3
                    % Zd = CNd × q × S / m  (alpha별)
                    y = CNd * q * SREF / m;
                    plot(alpha_pts, y, 'o-', ...
                        'Color',colors_mach(mi,:),'LineWidth',1.5,...
                        'DisplayName',mach_labels{mi});
                elseif pi==4
                    % Md = CMd × q × S × L / Iyy  (alpha별)
                    y = CMd * q * SREF * LREF / Iyy;
                    plot(alpha_pts, y, 'o-', ...
                        'Color',colors_mach(mi,:),'LineWidth',1.5,...
                        'DisplayName',mach_labels{mi});
                else
                    % Lp = CMQ/rad × q × S × L² / (2V × Iyy)
                    y = CMQ_rad * q * SREF * LREF^2 / (2*V*Iyy);
                    plot(alpha_pts, y*ones(nA,1), '-', ...
                        'Color',colors_mach(mi,:),'LineWidth',1.5,...
                        'DisplayName',mach_labels{mi});
                end
            end

            yline(0,'k--','LineWidth',0.8);
            xlabel('\alpha [deg]'); ylabel(deriv_titles{pi});
            title(sprintf('%s, %s',deriv_titles{pi},alt_labels{hi}));
            if hi==1 && pi==1
                legend('Location','best','FontSize',6);
            end
            grid on;
        end
    end
end

fprintf('\n=== 전체 완료 ===\n');