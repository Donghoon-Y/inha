% 지상국 위치 세계지도 표시
figure('Position', [100, 100, 1200, 600]);

% 세계 지도 배경
ax = worldmap('World');
land = shaperead('landareas', 'UseGeoCoords', true);
geoshow(ax, land, 'FaceColor', [0.85 0.90 0.85], 'EdgeColor', [0.6 0.6 0.6]);

% 바다 색상
setm(ax, 'FFaceColor', [0.75 0.85 0.95]);

% 지상국 데이터
gs_lat  = [37.39,  78.23,   69.66,  -72.02];
gs_lon  = [126.64, 15.41,   18.94,    2.53];
gs_name = {'InhaAeroSpace', 'KSAT Svalbard', 'KSAT Tromsø', 'KSAT TrollSat'};
colors  = [0.90 0.10 0.10;   % 빨강 - 인하대
           0.10 0.40 0.80;   % 파랑 - Svalbard
           0.10 0.40 0.80;   % 파랑 - Tromsø
           0.10 0.40 0.80];  % 파랑 - TrollSat

% 핀 찍기
for i = 1:4
    plotm(gs_lat(i), gs_lon(i), ...
        'Marker', '^', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', colors(i,:), ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2);
    
    % 라벨 오프셋 (겹침 방지)
    offset_lat = 4;
    offset_lon = 3;
    if i == 4  % TrollSat 남극 → 위로 표시
        offset_lat = 5;
    end
    
    textm(gs_lat(i) + offset_lat, gs_lon(i) + offset_lon, ...
        gs_name{i}, ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'Color', colors(i,:));
end


% 범례
legend_entries = [
    plot(nan, nan, '^', 'MarkerFaceColor', [0.90 0.10 0.10], 'MarkerEdgeColor', 'k', 'MarkerSize', 10),
    plot(nan, nan, '^', 'MarkerFaceColor', [0.10 0.40 0.80], 'MarkerEdgeColor', 'k', 'MarkerSize', 10)
];
legend(legend_entries, {'InhaAeroSpace (주 지상국)', 'KSAT 네트워크'}, ...
    'Location', 'southoutside', 'FontSize', 10);

% 저장
exportgraphics(gcf, 'groundstation_map.png', 'Resolution', 300);