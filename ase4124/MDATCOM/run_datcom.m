%% SM-2 DATCOM 자동 실행 스크립트
clc; clear;

%% 설정
work_dir   = '/Users/hoony/inha/ase4124/MDATCOM';
datcom_exe = '/opt/homebrew/bin/wine MDATCOM.EXE';
cd(work_dir);

input_files = {
    'cg1_input.txt',
    'cg2_input.txt',
    'cg1_delta_input.txt',       % DAMP → CMQ용
    'cg2_delta_input.txt',       % DAMP → CMQ용
    'cg1_trim_input.txt',  % TRIM → CM, CN용
    'cg2_trim_input.txt'   % TRIM → CM, CN용
};

output_files = {
    'cg1_output.txt',
    'cg2_output.txt',
    'cg1_delta_output.txt',
    'cg2_delta_output.txt',
    'cg1_trim_output.txt',
    'cg2_trim_output.txt'
};

%% 실행 루프
for i = 1:length(input_files)
    inp = input_files{i};
    out = output_files{i};

    fprintf('--------------------------------------------------\n');
    fprintf('[%d/%d] 실행 중: %s\n', i, length(input_files), inp);

    if ~isfile(inp)
        fprintf('[오류] 파일 없음: %s → 건너뜀\n', inp);
        continue;
    end

    copyfile(inp, 'for005.dat');

    cmd = ['printf "for005.dat\nfor006.dat\n" | ', datcom_exe];
    status = system(cmd);

    if status == 0
        if isfile('for006.dat')
            copyfile('for006.dat', out);
            fprintf('[완료] 결과 저장: %s\n', out);
        else
            fprintf('[경고] for006.dat 생성 안됨: %s\n', inp);
        end
    else
        fprintf('[오류] DATCOM 실행 실패 (status=%d): %s\n', status, inp);
    end
end

fprintf('--------------------------------------------------\n');
fprintf('=== 전체 완료: %d개 케이스 ===\n', length(input_files));