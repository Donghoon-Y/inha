%% Datcom Output 파싱 및 공력계수 플롯

clear; clc; close all;

%% ===== 파싱 =====
data_cg1 = parse_datcom('/Users/hoony/inha/ase4124/MDATCOM/cg1_output.txt');
data_cg2 = parse_datcom('/Users/hoony/inha/ase4124/MDATCOM/cg2_output.txt');
fprintf('CG1: %d 케이스  /  CG2: %d 케이스 파싱 완료\n',...
        length(data_cg1), length(data_cg2));

%% ===== 공통 설정 =====
mach_list = [0.5, 1.0, 1.5, 2.0, 2.5];
alt_list  = [0, 2, 5, 10, 20];
alt_names = {'ALT0','ALT2','ALT5','ALT10','ALT20'};
alt_str   = {'ALT=0m','ALT=2km','ALT=5km','ALT=10km','ALT=20km'};

% 물성치
S    = 0.092;   d    = 0.343;
mass = [707.618, 707.493];
Iyy  = [1063.369, 1645.475];
r2d  = 180/pi;

c_mach = [0.21 0.47 0.75;   % 파랑
           0.85 0.50 0.10;   % 주황
           0.17 0.63 0.17;   % 초록
           0.58 0.40 0.74;   % 보라
           0.77 0.31 0.32];  % 빨강
mk     = {'o','s','^','d','v'};

% 무차원 필드
nd_fields = {'CN','CM','CA','CNA','CMA','CMQ'};
nd_labels = {'CN','CM','CA','CNa [/deg]','CMa [/deg]','CMq [/deg]'};

% 유차원 필드
dm_fields = {'Z','M','X','Za','Ma','Mq'};
dm_labels = {'Z [N]','M [N·m]','X [N]','Za [m/s^2/rad]','Ma [1/s^2]','Mq [1/s]'};

data_all  = {data_cg1, data_cg2};
cg_prefix = {'CG1','CG2'};

%% ===== CG1, CG2 각각 고도별 Figure =====
for cg = 1:2
    dat = data_all{cg};
    m   = mass(cg);
    Iy  = Iyy(cg);

    for ai = 1:length(alt_list)
        caseid = [cg_prefix{cg} alt_names{ai}];
        cidx   = find_case(dat, caseid);

        fig_title = sprintf('%s  %s', cg_prefix{cg}, alt_str{ai});
        fig = figure();

        draw_one_alt_tab(fig, dat, cidx, nd_fields, nd_labels, ...
                         dm_fields, dm_labels, ...
                         mach_list, c_mach, mk, ...
                         m, Iy, S, d, r2d, ...
                         sprintf('%s (%s)  Mach별', cg_prefix{cg}, alt_str{ai}));
    end
end

%% ===== CG1 vs CG2 비교 Figure (ALT=0, 무차원) =====
i1 = find_case(data_cg1,'CG1ALT0');
i2 = find_case(data_cg2,'CG2ALT0');

fig = figure();
draw_compare_nd(fig, nd_fields, nd_labels, ...
    data_cg1, i1, data_cg2, i2, ...
    mach_list, c_mach, mk, ...
    'CG1 vs CG2  무차원  |  ALT=0m  |  실선=CG1  점선=CG2');

%% ===== CG1 vs CG2 비교 Figure (ALT=0, 유차원) =====
fig = figure();
draw_compare_dim(fig, dm_fields, dm_labels, ...
    data_cg1, i1, data_cg2, i2, ...
    mass, Iyy, S, d, r2d, ...
    mach_list, c_mach, mk, ...
    'CG1 vs CG2  유차원  |  ALT=0m  |  실선=CG1  점선=CG2');

fprintf('완료. Figure %d개 생성.\n', length(alt_list)*2 + 2);

%% ================================================================
%% 무차원(위) + 유차원(아래) 그리기
%% ================================================================
function draw_one_alt_tab(fig, dat, cidx, nd_f, nd_l, dm_f, dm_l, ...
                           mach_list, colors, mk, ...
                           m, Iy, S, d, r2d, ttl)
    nf = length(nd_f);  % 6
    nd = length(dm_f);  % 6
    total = nf + nd;    % 12 → 2x6 배치

    for fi = 1:nf
        ax = subplot(4, 3, fi, 'Parent', fig);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        lh=[]; ls={};
        for mi = 1:length(mach_list)
            % CMQ는 dynamic 필드
            field = nd_f{fi};
            [v,a] = get_field(dat, cidx, mach_list(mi), field);
            if ~isempty(v)
                h = plot(ax,a,v,['-' mk{mi}],'Color',colors(mi,:),...
                    'LineWidth',1.4,'MarkerSize',5);
                lh(end+1)=h; ls{end+1}=sprintf('M=%.1f',mach_list(mi)); %#ok<AGROW>
            end
        end
        xlabel(ax,'\alpha [deg]','Interpreter','tex');
        ylabel(ax, nd_l{fi},'Interpreter','none');
        title(ax,  nd_l{fi},'Interpreter','none','FontWeight','normal');
        set(ax,'XTick',[0 5 10 15 20]);
        if fi==1&&~isempty(lh)
            legend(ax,lh,ls,'Location','best','FontSize',7,'Interpreter','none');
        end
    end

    for fi = 1:nd
        ax = subplot(4, 3, nf+fi, 'Parent', fig);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        lh=[]; ls={};
        for mi = 1:length(mach_list)
            Mv  = mach_list(mi);
            blk = get_block(dat, cidx, Mv);
            if isempty(blk), continue; end
            q   = blk.q;    % DATCOM 출력에서 파싱한 동압 [N/m²]
            U0  = blk.vel;  % DATCOM 출력에서 파싱한 속도 [m/s]
            v = calc_one_dim(blk, dm_f{fi}, q, S, d, m, Iy, U0, r2d);
            if ~isempty(v) && ~all(v==0)
                h = plot(ax, blk.alpha, v, ['-' mk{mi}],'Color',colors(mi,:),...
                    'LineWidth',1.4,'MarkerSize',5);
                lh(end+1)=h; ls{end+1}=sprintf('M=%.1f',Mv); %#ok<AGROW>
            end
        end
        xlabel(ax,'\alpha [deg]','Interpreter','tex');
        ylabel(ax, dm_l{fi},'Interpreter','none');
        title(ax,  dm_l{fi},'Interpreter','none','FontWeight','normal');
        set(ax,'XTick',[0 5 10 15 20]);
        if fi==1&&~isempty(lh)
            legend(ax,lh,ls,'Location','best','FontSize',7,'Interpreter','none');
        end
    end

    sgtitle(fig, ttl, 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none');
end

%% ================================================================
%% CG1 vs CG2 무차원 비교
%% ================================================================
function draw_compare_nd(fig, nd_f, nd_l, dat1, idx1, dat2, idx2, ...
                          mach_list, colors, mk, ttl)
    c2 = [0.60 0.75 0.90;   % 연파랑
           0.95 0.76 0.50;   % 연주황
           0.60 0.85 0.60;   % 연초록
           0.82 0.72 0.90;   % 연보라
           0.95 0.65 0.65];  % 연빨강
    for fi = 1:length(nd_f)
        ax = subplot(2,3,fi,'Parent',fig);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        lh=[]; ls={};
        for mi = 1:length(mach_list)
            [v1,a1]=get_field(dat1,idx1,mach_list(mi),nd_f{fi});
            if ~isempty(v1)
                h=plot(ax,a1,v1,['-' mk{mi}],'Color',colors(mi,:),'LineWidth',1.4,'MarkerSize',5);
                lh(end+1)=h; ls{end+1}=sprintf('CG1 M=%.1f',mach_list(mi)); %#ok<AGROW>
            end
            [v2,a2]=get_field(dat2,idx2,mach_list(mi),nd_f{fi});
            if ~isempty(v2)
                h=plot(ax,a2,v2,['--' mk{mi}],'Color',c2(mi,:),'LineWidth',1.4,'MarkerSize',5);
                lh(end+1)=h; ls{end+1}=sprintf('CG2 M=%.1f',mach_list(mi)); %#ok<AGROW>
            end
        end
        xlabel(ax,'\alpha [deg]','Interpreter','tex');
        ylabel(ax,nd_l{fi},'Interpreter','none');
        title(ax,nd_l{fi},'Interpreter','none','FontWeight','normal');
        set(ax,'XTick',[0 5 10 15 20]);
        if fi==1&&~isempty(lh)
            legend(ax,lh,ls,'Location','best','FontSize',7,'NumColumns',2,'Interpreter','none');
        end
    end
    sgtitle(fig, ttl, 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none');
end

%% ================================================================
%% CG1 vs CG2 유차원 비교
%% ================================================================
function draw_compare_dim(fig, dm_f, dm_l, dat1, idx1, dat2, idx2, ...
                           mass, Iyy, S, d, r2d, ...
                           mach_list, colors, mk, ttl)
    colors2 = [0.60 0.75 0.90;   % 연파랑
             0.95 0.76 0.50;   % 연주황
             0.60 0.85 0.60;   % 연초록
             0.82 0.72 0.90;   % 연보라
             0.95 0.65 0.65];  % 연빨강
    for fi = 1:length(dm_f)
        ax = subplot(2,3,fi,'Parent',fig);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        lh=[]; ls={};
        for mi = 1:length(mach_list)
            Mv = mach_list(mi);
            for cg = 1:2
                if cg==1; dat=dat1; cidx=idx1; else; dat=dat2; cidx=idx2; end
                blk = get_block(dat,cidx,Mv);
                if isempty(blk), continue; end
                q   = blk.q;    % DATCOM 출력에서 파싱한 동압 [N/m²]
                U0  = blk.vel;  % DATCOM 출력에서 파싱한 속도 [m/s]
                v = calc_one_dim(blk,dm_f{fi},q,S,d,mass(cg),Iyy(cg),U0,r2d);
                if ~isempty(v)&&~all(v==0)
                    if cg==1; ls_='-'; col=colors(mi,:); else; ls_='--'; col=colors2(mi,:); end
                    h=plot(ax,blk.alpha,v,[ls_ mk{mi}],'Color',col,...
                        'LineWidth',1.4,'MarkerSize',5);
                    lh(end+1)=h; 
                    ls{end+1}=sprintf('CG%d M=%.1f',cg,Mv); 
                end
            end
        end
        xlabel(ax,'\alpha [deg]','Interpreter','tex');
        ylabel(ax,dm_l{fi},'Interpreter','none');
        title(ax,dm_l{fi},'Interpreter','none','FontWeight','normal');
        set(ax,'XTick',[0 5 10 15 20]);
        if fi==1&&~isempty(lh)
            legend(ax,lh,ls,'Location','best','FontSize',7,'NumColumns',2,'Interpreter','none');
        end
    end
    sgtitle(fig, ttl, 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none');
end

%% ================================================================
%% 유차원 단일 계수 계산
%% ================================================================
function v = calc_one_dim(blk, field, q, S, d, m, Iy, U0, r2d)
    n   = length(blk.alpha);
    v   = zeros(n, 1);      % 미리 zeros로 할당
    fF  = q * S;
    fMo = q * S * d;
    fZa = q * S / m;
    fMa = q * S * d / Iy;
    fMq = fMa * d / (2 * U0);

    switch field
        case 'Z'
            if ~isempty(blk.CN)
                v = fF * blk.CN;
            end
        case 'M'
            if ~isempty(blk.CM)
                v = fMo * blk.CM;
            end
        case 'X'
            if ~isempty(blk.CA)
                v = fF * blk.CA;
            end
        case 'Za'
            if ~isempty(blk.CNA)
                v = -fZa * blk.CNA * r2d;
            end
        case 'Ma'
            if ~isempty(blk.CMA)
                v = fMa * blk.CMA * r2d;
            end
        case 'Mq'
            if ~isempty(blk.CMQ)
                v = fMq * blk.CMQ * r2d;
            end
    end
end

%% ================================================================
%% 파싱
%% ================================================================
function data = parse_datcom(filename)
    fid=fopen(filename,'r');
    if fid<0, error('파일 없음: %s',filename); end
    raw={};
    while ~feof(fid), raw{end+1}=fgetl(fid); end 
    fclose(fid);
    data=struct(); cidx=0; i=1;
    while i<=length(raw)
        line=raw{i};
        if ~ischar(line), i=i+1; continue; end
        if ~isempty(regexp(line,'^\s*CASEID\s+\S','once'))&&...
           isempty(regexp(line,'^\s*\d+\s+CASEID','once'))
            tok=regexp(line,'CASEID\s+(\S+)','tokens');
            cidx=cidx+1;
            data(cidx).caseid=tok{1}{1};
            data(cidx).mach_blocks=struct([]);
            i=i+1; continue;
        end
        if cidx>0&&~isempty(regexp(line,'FLIGHT CONDITIONS','once'))
            for k=i+1:min(i+8,length(raw))
                if ~ischar(raw{k}), continue; end
                nums=sscanf(strtrim(raw{k}),'%f');
                if length(nums)>=4
                    mach_val = nums(1);
                    vel_val  = nums(3);   % 비행속도 [m/s]
                    q_val    = nums(4);   % 동압 [N/m²]
                    for mb=1:length(data(cidx).mach_blocks)
                        if abs(data(cidx).mach_blocks(mb).mach-mach_val)<0.01
                            data(cidx).mach_blocks(mb).q   = q_val;
                            data(cidx).mach_blocks(mb).vel = vel_val;
                            break;
                        end
                    end
                    break;
                end
            end
        end
        if cidx>0&&~isempty(regexp(line,'STATIC AERODYNAMICS FOR BODY','once'))
            mach_val=search_mach(raw,i,+1);
            for k=i+1:min(i+15,length(raw))
                if ~isempty(regexp(raw{k},'ALPHA\s+CN\s+CM\s+CA','once'))
                    sdata=[]; j=k+2;
                    while j<=length(raw)
                        nums=sscanf(strtrim(raw{j}),'%f');
                        if length(nums)>=11; sdata(end+1,:)=nums(1:11)'; 
                        else; break; end
                        j=j+1;
                    end
                    if ~isnan(mach_val)&&~isempty(sdata)
                        mb=length(data(cidx).mach_blocks)+1;
                        data(cidx).mach_blocks(mb).mach  = mach_val;
                        data(cidx).mach_blocks(mb).alpha = sdata(:,1);
                        data(cidx).mach_blocks(mb).CN    = sdata(:,2);
                        data(cidx).mach_blocks(mb).CM    = sdata(:,3);
                        data(cidx).mach_blocks(mb).CA    = sdata(:,4);
                        data(cidx).mach_blocks(mb).CY    = sdata(:,5);
                        data(cidx).mach_blocks(mb).CLN   = sdata(:,6);
                        data(cidx).mach_blocks(mb).CLL   = sdata(:,7);
                        data(cidx).mach_blocks(mb).CNA   = sdata(:,8);
                        data(cidx).mach_blocks(mb).CMA   = sdata(:,9);
                        data(cidx).mach_blocks(mb).CYB   = sdata(:,10);
                        data(cidx).mach_blocks(mb).CLNB  = sdata(:,11);
                        data(cidx).mach_blocks(mb).CNQ   = [];
                        data(cidx).mach_blocks(mb).CMQ   = [];
                        data(cidx).mach_blocks(mb).CAQ   = [];
                        data(cidx).mach_blocks(mb).CNAD  = [];
                        data(cidx).mach_blocks(mb).CMAD  = [];
                        data(cidx).mach_blocks(mb).CYR   = [];
                        data(cidx).mach_blocks(mb).CLNR  = [];
                        data(cidx).mach_blocks(mb).CLLR  = [];
                    end
                    break;
                end
            end
        end
        if cidx>0&&~isempty(regexp(line,'DYNAMIC DERIVATIVES \(PER DEGREE\)','once'))
            mach_val=search_mach(raw,i,-1);
            ddata=[]; j=i+2;
            while j<=length(raw)
                dline=strtrim(raw{j});
                if isempty(dline)||~isempty(regexp(dline,'^[A-Za-z\*]','once')); break; end
                nums=sscanf(dline,'%f');
                if length(nums)>=6
                    row=nan(1,12); row(1:length(nums))=nums(1:min(12,end))';
                    ddata(end+1,:)=row; %#ok<AGROW>
                end
                j=j+1;
            end
            if ~isnan(mach_val)&&~isempty(ddata)
                for mb=1:length(data(cidx).mach_blocks)
                    if abs(data(cidx).mach_blocks(mb).mach-mach_val)<0.01
                        data(cidx).mach_blocks(mb).CNQ  = ddata(:,2);
                        data(cidx).mach_blocks(mb).CMQ  = ddata(:,3);
                        data(cidx).mach_blocks(mb).CAQ  = ddata(:,4);
                        data(cidx).mach_blocks(mb).CNAD = ddata(:,5);
                        data(cidx).mach_blocks(mb).CMAD = ddata(:,6);
                        if size(ddata,2)>=9
                            data(cidx).mach_blocks(mb).CYR  = ddata(:,7);
                            data(cidx).mach_blocks(mb).CLNR = ddata(:,8);
                            data(cidx).mach_blocks(mb).CLLR = ddata(:,9);
                        end
                        break;
                    end
                end
            end
        end
        i=i+1;
    end
end

function m=search_mach(raw,i,dir)
    m=NaN;
    if dir>0; rng=i+1:min(i+14,length(raw));
    else;      rng=max(1,i-14):i; end
    for k=rng
        if ~ischar(raw{k}), continue; end
        tok=regexp(raw{k},'^\s*(\d+\.\d+)\s+\d','tokens');
        if ~isempty(tok), m=str2double(tok{1}{1}); return; end
    end
end

function idx=find_case(data,keyword)
    idx=[];
    for i=1:length(data)
        if strcmp(data(i).caseid, keyword), idx=i; return; end
    end
end

function [vals,alphas]=get_field(data,cidx,mach_val,field)
    vals=[]; alphas=[];
    if isempty(cidx)||cidx>length(data), return; end
    for mb=1:length(data(cidx).mach_blocks)
        blk=data(cidx).mach_blocks(mb);
        if abs(blk.mach-mach_val)<0.01&&isfield(blk,field)&&~isempty(blk.(field))
            vals=blk.(field); alphas=blk.alpha; return;
        end
    end
end

function blk=get_block(data,cidx,mach_val)
    blk=[];
    if isempty(cidx)||cidx>length(data), return; end
    for mb=1:length(data(cidx).mach_blocks)
        if abs(data(cidx).mach_blocks(mb).mach-mach_val)<0.01
            blk=data(cidx).mach_blocks(mb); return;
        end
    end
end