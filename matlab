clc;
clear;

% ==========================================================
% IMPORTANTE:
% NO USAR close all
% porque queremos conservar las trayectorias anteriores.
% ==========================================================

figure(1);
hold on;

% =========================================================
% CARGAR ENTORNO DETECTADO POR PYTHON
% =========================================================

carpetaScript = fileparts(mfilename('fullpath'));
archivoVision = fullfile(carpetaScript, 'entorno_detectado (1).mat');

if ~isfile(archivoVision)
    error('No se encontro el archivo: %s', archivoVision);
end

datosVision = load(archivoVision);


disp('========================================');
disp(' DATOS RECIBIDOS DESDE PYTHON');
disp('========================================');

disp('Inicio XY [mm]:');
disp(datosVision.start_xy_mm);

disp('Meta XY [mm]:');
disp(datosVision.goal_xy_mm);

disp('Inicio obstaculo XY [mm]:');
disp(datosVision.obstacleStartXY_mm);

disp('Tamano obstaculo XY [mm]:');
disp(datosVision.obstacleLengthXY_mm);


%% =========================================================
% DIMENSIONES DEL ESPACIO
% ==========================================================

mapX = round(datosVision.workspace_x_mm(1));
mapY = round(datosVision.workspace_y_mm(1));

mapZ = 15;


fprintf('\nMapa solicitado:\n');
fprintf('X = %d mm\n', mapX);
fprintf('Y = %d mm\n', mapY);
fprintf('Z = %d mm\n', mapZ);


%% =========================================================
% CREAR MAPA 3D VACIO
% ==========================================================

threeDimMap = zeros( ...
    mapX, ...
    mapY, ...
    mapZ ...
);


%% =========================================================
% DATOS DEL OBSTACULO
% ==========================================================

obsX = round( ...
    datosVision.obstacleStartXY_mm(1) ...
);

obsY = round( ...
    datosVision.obstacleStartXY_mm(2) ...
);


obsLargoX = round( ...
    datosVision.obstacleLengthXY_mm(1) ...
);

obsLargoY = round( ...
    datosVision.obstacleLengthXY_mm(2) ...
);


%% =========================================================
% ALTURA PROVISIONAL DEL OBSTACULO
% ==========================================================

alturaObstaculo = 15;


%% =========================================================
% LIMITES DEL OBSTACULO
% ==========================================================

x1 = max(1, obsX);
y1 = max(1, obsY);

x2 = min( ...
    mapX, ...
    x1 + obsLargoX - 1 ...
);

y2 = min( ...
    mapY, ...
    y1 + obsLargoY - 1 ...
);

z1 = 1;

z2 = min( ...
    mapZ, ...
    alturaObstaculo ...
);


%% =========================================================
% COLOCAR OBSTACULO EN EL MAPA
% ==========================================================

threeDimMap( ...
    x1:x2, ...
    y1:y2, ...
    z1:z2 ...
) = 1;


%% =========================================================
% CREAR mapData
% ==========================================================

mapData = struct();

mapData.threeDimMap = ...
    threeDimMap;


mapData.obstacleStartVertex = [ ...
    x1, ...
    y1, ...
    0 ...
];


tamanoRealX = x2 - x1 + 1;
tamanoRealY = y2 - y1 + 1;


mapData.obstacleLenghtXYZ = [ ...
    tamanoRealX, ...
    tamanoRealY, ...
    alturaObstaculo ...
];


%% =========================================================
% INFORMACION DEL MAPA
% ==========================================================

mapSize = size( ...
    mapData.threeDimMap ...
);


fprintf('\n========================================\n');
fprintf('MAPA GENERADO\n');
fprintf('========================================\n');

fprintf( ...
    'Mapa: %d x %d x %d\n', ...
    mapSize(1), ...
    mapSize(2), ...
    mapSize(3) ...
);


fprintf('\nOBSTACULO:\n');

fprintf('X inicial = %d mm\n', x1);
fprintf('Y inicial = %d mm\n', y1);
fprintf('Tamano X  = %d mm\n', tamanoRealX);
fprintf('Tamano Y  = %d mm\n', tamanoRealY);
fprintf('Altura Z  = %d mm\n', alturaObstaculo);


%% =========================================================
% START
% ==========================================================

startX = round( ...
    datosVision.start_xy_mm(1) ...
);

startY = round( ...
    datosVision.start_xy_mm(2) ...
);


startX = max( ...
    1, ...
    min(mapX,startX) ...
);

startY = max( ...
    1, ...
    min(mapY,startY) ...
);


startZ = 8;


start_vertex = [ ...
    startX, ...
    startY, ...
    startZ ...
];


%% =========================================================
% GOAL
% ==========================================================

goalX = round( ...
    datosVision.goal_xy_mm(1) ...
);

goalY = round( ...
    datosVision.goal_xy_mm(2) ...
);


goalX = max( ...
    1, ...
    min(mapX,goalX) ...
);

goalY = max( ...
    1, ...
    min(mapY,goalY) ...
);


goalZ = 8;


goal_vertex = [ ...
    goalX, ...
    goalY, ...
    goalZ ...
];


%% =========================================================
% MOSTRAR START Y GOAL
% ==========================================================

fprintf('\n========================================\n');
fprintf('PUNTOS DEL RRT*\n');
fprintf('========================================\n');


fprintf( ...
    'START = [%d  %d  %d]\n', ...
    start_vertex(1), ...
    start_vertex(2), ...
    start_vertex(3) ...
);


fprintf( ...
    'GOAL  = [%d  %d  %d]\n', ...
    goal_vertex(1), ...
    goal_vertex(2), ...
    goal_vertex(3) ...
);


%% =========================================================
% VERIFICAR START
% ==========================================================

if mapData.threeDimMap( ...
        start_vertex(1), ...
        start_vertex(2), ...
        start_vertex(3)) == 1

    error( ...
        'ERROR: El START esta dentro del obstaculo.' ...
    );

end


%% =========================================================
% VERIFICAR GOAL
% ==========================================================

if mapData.threeDimMap( ...
        goal_vertex(1), ...
        goal_vertex(2), ...
        goal_vertex(3)) == 1

    error( ...
        'ERROR: El GOAL esta dentro del obstaculo.' ...
    );

end


%% =========================================================
% PARAMETROS RRT*
% ==========================================================

K = 4000;

% Paso grande = menos waypoints
u = 20;

p = 0.05;

minDis = ...
    2 * u * sin(pi/8);

R = ...
    2 * u;


%% =========================================================
% PARAMETROS DINAMICOS
% ==========================================================

rN = [4,4,1];

sN = 500;

lim = [0.6,1.4];

mid = 1;

type = 3;

k = 1;

pdf = [0,1];


%% =========================================================
% VARIABLE GLOBAL
% ==========================================================

global obstacleNum


%% =========================================================
% NUMERO DE EJECUCIONES
% ==========================================================

maxTimes = 1;


%% =========================================================
% CREAR CARPETA DE DATOS
% ==========================================================

if ~exist('dataFiles','dir')

    mkdir('dataFiles');

end


%% =========================================================
% CARPETA DEL CSV PARA PYTHON / FANUC
% ==========================================================

carpetaCSV = ...
    'D:\Matlab_Fanuc\TS-RIL-melodic-devel\SRRT_star\Code\Fanuc_GESRTP_Driver-main';


if ~exist(carpetaCSV,'dir')

    mkdir(carpetaCSV);
1616
end


archivoCSV = fullfile( ...
    carpetaCSV, ...
    'waypoints_1.csv' ...
);


fprintf('\nCSV se guardara en:\n');
fprintf('%s\n',archivoCSV);


%% =========================================================
% EJECUTAR RRT*
% ==========================================================

for T = 1:maxTimes

    obstacleNum = 0;


    %% -----------------------------------------------------
    % INICIAR TIEMPO
    % ------------------------------------------------------

    tic;


    %% -----------------------------------------------------
    % PARAMETROS RRT
    % ------------------------------------------------------

    RRT_param = struct( ...
        'iterativeNum',K, ...
        'expandDis',u, ...
        'goalTrend',p, ...
        'minDistance',minDis, ...
        'regionRadius',R, ...
        'goalScaleK',k ...
    );


    %% -----------------------------------------------------
    % PARAMETROS DINAMICOS
    % ------------------------------------------------------

    dynamic_param = struct( ...
        'regionNum',rN, ...
        'sampleNum',sN, ...
        'scaleLim',lim, ...
        'midScale',mid, ...
        'pdfParam',pdf, ...
        'isAdjustScale',type ...
    );


    %% -----------------------------------------------------
    % EJECUTAR ALGORITMO
    % ------------------------------------------------------

    [ ...
        flag, ...
        vertex_param, ...
        optimalPath, ...
        minCost ...
    ] = RRTstar_Algorithm( ...
        mapData.threeDimMap, ...
        RRT_param, ...
        start_vertex, ...
        goal_vertex, ...
        dynamic_param ...
    );


    elapsedTime = toc;


    %% =====================================================
    % SI NO ENCUENTRA CAMINO
    % ======================================================

    if flag == 0

        disp(' ');
        disp('NO SE ENCONTRO UNA RUTA FACTIBLE.');

        return;

    end


    %% =====================================================
    % EXTRAER WAYPOINTS DE optimalPath
    % ======================================================

    pathNum = length( ...
        optimalPath ...
    );


    pathXYZ = zeros( ...
        pathNum, ...
        3 ...
    );


    for i = 1:pathNum

        pathXYZ(i,:) = ...
            optimalPath(i).vertex;

    end


    %% =====================================================
    % ASEGURAR ORDEN:
    %
    % START -> ... -> GOAL
    % ======================================================

    distanciaInicioPrimero = norm( ...
        pathXYZ(1,:) - start_vertex ...
    );


    distanciaInicioUltimo = norm( ...
        pathXYZ(end,:) - start_vertex ...
    );


    if distanciaInicioUltimo < distanciaInicioPrimero

        pathXYZ = flipud( ...
            pathXYZ ...
        );

    end


    %% =====================================================
    % MOSTRAR COORDENADAS
    % ======================================================

    disp(' ');
    disp('========================================');
    disp(' WAYPOINTS DE LA TRAYECTORIA');
    disp('========================================');


    for i = 1:pathNum

        fprintf( ...
            'P%d = [%.3f  %.3f  %.3f]\n', ...
            i, ...
            pathXYZ(i,1), ...
            pathXYZ(i,2), ...
            pathXYZ(i,3) ...
        );

    end


    %% =====================================================
    % GENERAR CSV
    % ======================================================

    writematrix( ...
        pathXYZ, ...
        archivoCSV ...
    );


    disp(' ');
    disp('========================================');
    disp(' CSV GENERADO');
    disp('========================================');

    fprintf( ...
        'Numero de puntos: %d\n', ...
        pathNum ...
    );

    fprintf( ...
        'Archivo:\n%s\n', ...
        archivoCSV ...
    );


    %% =====================================================
    % DIBUJAR SIN BORRAR TRAYECTORIAS ANTERIORES
    % ======================================================

    if T == maxTimes

        % Siempre utilizar la misma figura
        figure(1);

        % Conservar dibujo anterior
        hold on;


        RRT_Plot( ...
            mapData, ...
            start_vertex, ...
            goal_vertex, ...
            vertex_param, ...
            optimalPath ...
        );


        % Mantener activado para la siguiente ejecucion
        hold on;


        title( ...
            'Trayectorias RRT*' ...
        );

    end


    %% =====================================================
    % RESULTADOS
    % ======================================================

    vertexNum = length( ...
        vertex_param ...
    );


    disp(' ');
    disp('------------- Result -------------');


    fprintf( ...
        'elapsedTime: %0.3f sec\n', ...
        elapsedTime ...
    );


    fprintf( ...
        'vertexNum:   %d\n', ...
        vertexNum ...
    );


    fprintf( ...
        'minPathCost: %0.3f\n', ...
        minCost ...
    );


    fprintf( ...
        'obstacleNum: %d\n', ...
        obstacleNum ...
    );


    disp('----------------------------------');


    %% =====================================================
    % ARCHIVOS DE EXPERIMENTO
    % ======================================================

    filename1 = sprintf( ...
        '.\\dataFiles\\experimentalData0%d.txt', ...
        type ...
    );


    filename2 = ...
        '.\\dataFiles\\experimentalDataAverage.txt';


    dataGroupNumMax = 10;


    if T == 1

        isDeleteOldData = 1;

    else

        isDeleteOldData = 0;

    end


    %% =====================================================
    % GUARDAR DATOS
    % ======================================================

    saveData( ...
        [ ...
            elapsedTime, ...
            vertexNum, ...
            minCost, ...
            obstacleNum ...
        ], ...
        filename1, ...
        isDeleteOldData ...
    );


    %% =====================================================
    % LEER DATOS
    % ======================================================

    data = extractData( ...
        filename1, ...
        4 ...
    );


    dataSize = size( ...
        data ...
    );


    dataAverage = calAverage( ...
        data, ...
        1 ...
    );


    %% =====================================================
    % PROMEDIOS
    % ======================================================

    tmp = extractData( ...
        filename2, ...
        4 ...
    );


    if isempty(tmp)

        tmp = zeros( ...
            4, ...
            dataSize(2) ...
        );

    end


    tmp(type + 1,:) = ...
        dataAverage;


    saveData( ...
        tmp, ...
        filename2, ...
        1 ...
    );


    %% =====================================================
    % INFORMACION
    % ======================================================

    fprintf( ...
        'Type-%d: This is the %d-th (Max: %d) experiment.\n', ...
        type, ...
        dataSize(1), ...
        dataGroupNumMax ...
    );


    %% =====================================================
    % RESULTADOS PROMEDIO
    % ======================================================

    if dataSize(1) >= dataGroupNumMax

        disp(' ');
        disp('----------- All Result -----------');


        fprintf( ...
            'Average elapsedTime: %0.3f sec\n', ...
            dataAverage(1) ...
        );


        fprintf( ...
            'Average vertexNum:   %d\n', ...
            round(dataAverage(2)) ...
        );


        fprintf( ...
            'Average minPathCost: %0.3f\n', ...
            dataAverage(3) ...
        );


        fprintf( ...
            'Average obstacleNum: %d\n', ...
            round(dataAverage(4)) ...
        );


        disp('----------------------------------');

    end

end


%% =========================================================
% FINAL
% ==========================================================

disp(' ');
disp('========================================');
disp(' RRT* FINALIZADO');
disp('========================================');

fprintf('\nWaypoints guardados en:\n');
fprintf('%s\n',archivoCSV);

% ==========================================================
% NO PONER:
%
% close all
% hold off
%
% si quieres conservar las trayectorias anteriores.
% ==========================================================
