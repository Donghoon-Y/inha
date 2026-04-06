%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% with Flight Time Estimation
%
% M. Biczyski, R. Sehab, G.Krebs, J.F. Whidborne, P. Luk
%
% load_propPerf.m - implements function load_propPerf() that reads static 
% propeller performance from the database; requires file 
% 'PER2_STATIC-2.DAT' from APC Performance Database Summary in
% 'PERFILES_WEB' folder
%% ------------------------------------------------------------------------

% output = RPM, Thrust (g), Power (W), Torque (Nm), Cp, Ct
function output = load_propPerf(name, bPlot)
%% 'PERFILES_WEB/PER2_STATIC-2.DAT'라는 데이터베이스 파일을 열기. fopen은 텍스트 파일을 여는 것이고, 'rt'는 read text, 텍스트 읽기모드
%% 전반적인 과정
%% 일단 우리가 선별한 프로펠러의 이름과 알맞은 'PER3_10x6.dat' 파일을 가져옴.
%% 10x6.dat부터 읽어서 우리가 성능 지표에서 여기에 맞는 텍스트를 찾음.
%% 텍스트를 찾고 나면 4줄은 헤더라서 생략하고 그 다음부터 수치들을 전부 기록
%% 기록하다가 빈줄이나 파일 끝이 나오면 temp2 = -1이 나올 것이고, 그러면서 종료, 이거를 for 문 돌려서 전체 기록할 수 있게끔 함.
    fileID = fopen('APC_Prop/PER2_STATIC-2.DAT', 'rt'); % access the data file % text 파일을 열기 위해서 고유한 식별자를 부여.
    
    temp1 = fgetl(fileID); % 그 파일의 첫 줄을 읽기.
    while ~strcmp(sscanf(temp1, '%s'), name(6:end)) % look for the propeller specified by name %propList_considered{ii,2}에서 고유한 프롭 이름.
        % 'PER3_10x6.dat'에서 10x6.dat을 가져옴. 그러니까 proplist_considered는 우리가
        % 생각하고 있는 프롭이므로 거기서 우리가 알고 싶은 성능을 가진 프롭에 맞는 이름을
        % 가져오는게 맞음.
        temp1 = fgetl(fileID);
    end
    
    for ii = 1:4 % skip blank space & headings % 실제 숫자 표부터 시작하기 위함. 헤더 제외하기 위함.
        fgetl(fileID);
    end
    
    currentLine = sscanf(fgetl(fileID), '%f %f %f %f %f %f'); % read first line of data
    staticData = [];
    while ~isempty(currentLine) % read next lines of data until blank space or end of file
        staticData(end+1, :) = currentLine;
        temp2 = fgetl(fileID);
        if temp2 == -1
            temp2 = '';
        end
        currentLine = sscanf(temp2, '%f %f %f %f %f %f');
    end
    fclose(fileID); % close data file
    
    staticData(:,2) = staticData(:,2)*4.4482216/9.81*1000; % convert thrust values to gram-force
    staticData(:,3) = staticData(:,3)*746; % convert power values to Watt
    staticData(:,4) = staticData(:,4)*0.112985; % convert torque values to Newton-meter
    
    if bPlot % (optional) plot static propeller characteristics
        figure;
        subplot(2,1,1);
        yyaxis left;
        plot(staticData(:,1), staticData(:,2));
        xlabel('RPM');
        ylabel('Thrust (g)');
        ylim([0 1.1*max(staticData(:,2))]);
        title([name(6:end-4) ' - Static Thrust / Torque']);
        yyaxis right;
        plot(staticData(:,1), staticData(:,4));
        ylabel('Torque (Nm)');
        ylim([0 1.1*max(staticData(:,4))]);
        grid;
        
        subplot(2,1,2);
        yyaxis left;
        plot(staticData(:,1), staticData(:,6));
        xlabel('RPM');
        ylabel('Thrust Coefficient');
        ylim([0 1.1*max(staticData(:,6))]);
        title([name(6:end-4) ' - Thrust / Power Coefficients']);
        yyaxis right;
        plot(staticData(:,1), staticData(:,5));
        ylabel('Power Coefficient');
        ylim([0 1.1*max(staticData(:,5))]);
        grid;
    end
    
    output = staticData;
end