% Aquisição e plotagem de dados dos sensores do aparelho celular 
% ========================================================================
% Autor: Christian Danner Ramos de Carvalho
% Data: 26/10/2024
%=========================================================================

close all
clear 
clc

% Crie um objeto mobiledev no MATLAB para acessar os dados dos sensores
m = mobiledev('Redmi Note 13 Pro 5G');

% Ative os sensores que deseja usar
% m.AccelerationSensorEnabled = true;
% m.AngularVelocitySensorEnabled = true;
% m.OrientationSensorEnabled = true;
% m.PositionSensorEnabled = true;

m.Logging = true; % Start no log dos sensores

% Cria uma figura para exibir os dados em tempo real
figure;

while true
    % Captura os dados dos sensores
    [accelData, accelTime] = accellog(m);
    [angVelData, angVelTime] = angvellog(m);
    [orientationData, orientationTime] = orientlog(m);
    % [lat, lon, alt, posTime] = poslog(m);

    % Limpa a figura para atualização
    clf;

    % Plota os dados de aceleração
    subplot(2, 2, 1);
    plot(accelTime, accelData);
    grid on;
    title('Aceleração');
    xlabel('Tempo (s)');
    ylabel('m/s^2');
    legend('X', 'Y', 'Z');

    % Plota os dados de velocidade angular
    subplot(2, 2, 2);
    plot(angVelTime, angVelData);
    grid on;
    title('Velocidade Angular');
    xlabel('Tempo (s)');
    ylabel('rad/s');
    legend('X', 'Y', 'Z');

    % Plota os dados de orientação
    subplot(2, 2, 3);
    plot(orientationTime, orientationData);
    grid on;
    title('Orientação');
    xlabel('Tempo (s)');
    ylabel('Graus');
    legend('Yaw', 'Pitch', 'Roll');

    % % Plota os dados de posição
    % subplot(2, 3, 4);
    % plot(posTime, [lat, lon, alt]);
    % title('Posição');
    % xlabel('Tempo (s)');
    % ylabel('Latitude/Longitude/Altitude');
    % legend('Latitude', 'Longitude', 'Altitude');

    % Plota a orientação do aparelho em 3D
    hBody = subplot(2, 2, 4);
    axis equal;
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Orientação do Aparelho em 3D');
    grid on;
    view(3);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);

    % Inicializa os vetores de orientação (setas)
    X_arrow = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2); % Eixo X em vermelho
    Y_arrow = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2); % Eixo Y em verde
    Z_arrow = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2); % Eixo Z em azul

    % Define as coordenadas do prisma (representando o celular)
    prismVertices = [
        -0.2 -0.4 -0.05;  % Vértice 1
        0.2 -0.4 -0.05;  % Vértice 2
        0.2  0.4 -0.05;  % Vértice 3
        -0.2  0.4 -0.05;  % Vértice 4
        -0.2 -0.4  0.05;  % Vértice 5
        0.2 -0.4  0.05;  % Vértice 6
        0.2  0.4  0.05;  % Vértice 7
        -0.2  0.4  0.05;  % Vértice 8
        ];

    % Define as faces do prisma
    prismFaces = [
        1 2 3 4;  % Face inferior
        5 6 7 8;  % Face superior
        1 2 6 5;  % Face lateral 1
        2 3 7 6;  % Face lateral 2
        3 4 8 7;  % Face lateral 3
        4 1 5 8;  % Face lateral 4
        ];

    % Desenha o prisma
    prism = patch('Vertices', prismVertices, 'Faces', prismFaces, ...
        'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'k');

    % Captura os dados de orientação em tempo real
    orientationData = m.Orientation;
    
    % Verifica se há dados válidos
    if ~isempty(orientationData) && numel(orientationData) == 3
        % Extrai os ângulos de Euler (Yaw, Pitch, Roll)
        yaw = deg2rad(-orientationData(1)+136.75);
        pitch = deg2rad(-orientationData(2));
        roll = deg2rad(orientationData(3));
        
        % Converte os ângulos de Euler para matriz de rotação
        rotm = eul2rotm([yaw, pitch, roll], 'ZXY');
        
        % Atualiza as setas dos eixos com base na matriz de rotação
        set(X_arrow, 'UData', rotm(1, 1), 'VData', rotm(2, 1), 'WData', rotm(3, 1));
        set(Y_arrow, 'UData', rotm(1, 2), 'VData', rotm(2, 2), 'WData', rotm(3, 2));
        set(Z_arrow, 'UData', rotm(1, 3), 'VData', rotm(2, 3), 'WData', rotm(3, 3));

        % Aplica a rotação ao prisma
        rotatedVertices = (rotm * prismVertices')';
        set(prism, 'Vertices', rotatedVertices);
    end

    % Atualiza a exibição
    drawnow;

    % Adiciona uma pausa para controlar a taxa de atualização
    pause(0.01);
end


