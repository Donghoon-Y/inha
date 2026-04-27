% output = name, file, diameter (in), pitch (in), mass (g)
function output = load_propList()
    [~, ~, everything] = xlsread('APC_Prop/WEBLIST.xlsx'); 
    
    % fileList를 6열 크기로 미리 할당 (시뮬레이션 속도 향상을 위한 사전 할당)
    fileList = cell(size(everything,1)-5, 6); 
    
    for ii = 6:size(everything,1) % skip first 5 lines
        % 오류 수정 부분: 전체 행(:)이 아닌 1열과 2열(1:2)에만 대입하도록 수정
        fileList(ii-5, 1:2) = [everything(ii,2) everything(ii,1)]; 
    end
    clear everything; 
    
%% 2026년 데이터 파일의 'PRODUCT LIST' 시트 데이터를 읽어오도록 수정
    [~, ~, everything] = xlsread('PROP-DATA-FILE_202602.xlsx', 'PRODUCT LIST'); 
    
    for ii = 1:size(fileList,1) 
        for jj = 2:size(everything,1)
            % 빈 셀이거나 숫자가 들어있는 경우 startsWith에서 에러가 나는 것을 방지
            if ischar(everything{jj, 1}) || isstring(everything{jj, 1})
                if startsWith(everything{jj, 1}, fileList{ii,1})      
                    
                    % Diameter (8열)
                    diam = everything{jj, 8};
                    if ischar(diam) || isstring(diam), diam = str2double(diam); end
                    
                    % Pitch (7열)
                    pitch = everything{jj, 7};
                    if ischar(pitch) || isstring(pitch), pitch = str2double(pitch); end
                    
                    % Mass (16열)
                    mass = everything{jj, 16};
                    if ischar(mass) || isstring(mass), mass = str2double(mass); end
                    
                    % 추출한 값을 fileList의 3~5열에 저장
                    fileList(ii, 3:5) = {diam, pitch, mass}; 
                    break;
                end
            end
        end
    end
    
    fileList(cellfun('isempty', fileList(:,3)), :) = []; % remove entries without parameters
    
    % RPM limits
    for ii = 1:size(fileList, 1) 
        temp1 = fileList{ii,1};  
        temp2 = temp1(isstrprop(temp1,'alpha')); 
        switch temp2(2:end)         
            case 'PN'
                SpeedLimitSet = 190000;
            case {'W' 'N' 'P'}
                SpeedLimitSet = 270000;
            case {'E' 'F' 'WE' 'EPN' 'ECD'}
                SpeedLimitSet = 145000;
            case 'MR'
                SpeedLimitSet = 105000;
            case 'SF'
                SpeedLimitSet = 65000;
            case {'E-3' 'E-4' 'C'}
                SpeedLimitSet = 270000;
            otherwise
                SpeedLimitSet = 225000;
        end
        fileList{ii,6} = SpeedLimitSet / fileList{ii,3};
    end
    output = fileList;
end