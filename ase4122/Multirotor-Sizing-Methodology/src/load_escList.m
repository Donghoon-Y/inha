function escList = load_escList()
%% load_escList - ESC_data.xlsx 파일을 읽어 셀 배열로 반환
%
% 반환값:
%   escList - N x 4 셀 배열
%             열 1: ID (정수)
%             열 2: ESC명 (문자열)
%             열 3: 허용 전류 A (숫자)
%             열 4: 무게 g (숫자)

thisDir  = fileparts(mfilename('fullpath'));
filePath = fullfile(thisDir, '..', 'ESC_list', 'ESC_data.xlsx');

T = readtable(filePath, 'Sheet', 1, 'VariableNamingRule', 'preserve');

nRows = height(T);
escList = cell(nRows, 4);

for i = 1:nRows
    escList{i, 1} = i;
    escList{i, 2} = T{i, 1}{1};   % ESC명
    escList{i, 3} = T{i, 2};      % 허용 전류 (A)
    escList{i, 4} = T{i, 3};      % 무게 (g)
end

end