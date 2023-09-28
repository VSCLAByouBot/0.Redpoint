clc,clear,close all;

%% Function Button ==================================================
Optimal_Fun = 0;
DH_switch = 0;

%% Load Data ========================================================
% Robot_EEF for Robot, Camera_EEF for Camera
% H2E for Hand 2 Eye, E2H for Eye 2 Hand

tmp        = load(['..\Robot_Data\MeasuredData_230912.txt'])';   % Robot [Pos, Vel, Tor]

index      = [1:4,6:9,11:12];                              % 選擇用哪幾張照片
N          = length(index);                         % Number Of Data

RobotJoint = tmp(1 : 6, index);

Date       = '20221124';
% fd         = ['..\RedPoint_Detection\RedPoint_Detection\Pic\', Date];
% Camera_EEF = load([fd, '\Camera_EEF.txt'])';     % Camera coordinate  [X; Y; Z]
Camera_EEF = load(['..\Camera_Data\2023_9_12_22_37_2_redpoint.txt'])';     % Camera coordinate  [X; Y; Z]
Camera_EEF = Camera_EEF(:, [index]);

%% Forward Kinematics ===============================================
%(theta) (D)             (a)      (alpha)
% My Robot with stick
DH_table = [ 0      345             75       pi/2    ;
    pi/2   0               270      0       ;
    0      0               90       pi/2    ;
    0      295             0       -pi/2    ;
    0      0               0        pi/2    ;
    0      102 + 135       0        0       ; ] ; 

% My Robot with Nozzle
% DH_table = [ 0      345             75       pi/2    ;
%     pi/2   0               270      0       ;
%     0      0               90       pi/2    ;
%     0      295             0       -pi/2    ;
%     0      0               0        pi/2    ;
%     0      102 + 125       0        0       ; ] ; 

% DH_table = [ 0      374          25       -pi/2    ;
%             -pi/2   0            340      0       ;
%             0       0             40       -pi/2    ;
%             0       345           0       pi/2    ;
%             0       0             0        -pi/2    ;
%             0       74            0        0       ; ] ;


Robot_EEF = zeros(3, N);

for i = 1 : N
    [ ~, ~, Robot_EEF(:, i) ] = ForwardKinemetics( DH_table , RobotJoint(:, i));
end

Camera_EEF(1, :)	= Camera_EEF(1, :);
[E2H, H2E]          = Transform_Matrix_SVD(Camera_EEF, Robot_EEF, N);

% rotation matrix ---------------------------------------------------------
ShowTransFormation(E2H, 'Eye 2 Hand')
ShowTransFormation(H2E, 'Hand 2 Eye')

% Data Display ------------------------------------------------------------
DisplayData(Robot_EEF, E2H * [Camera_EEF; ones(1, N)], 'Robot Coordinate',  'Robot_EEF', 'E2H')
DisplayData(Camera_EEF, H2E * [Robot_EEF; ones(1, N)], 'Camera Coordinate', 'Camera_EEF', 'H2E')
 
% Data Plot ---------------------------------------------------------------
figure('name', 'Data Display')
subplot(2, 1, 1), DataPlot(Robot_EEF, E2H * [Camera_EEF; ones(1, N)], {'Robot Coordinate', 'Eye 2 Hand'})
subplot(2, 1, 2), DataPlot(Camera_EEF, H2E * [Robot_EEF; ones(1, N)], {'Camera Coordinate', 'Hand 2 Eye'})

set(gcf, 'Position', [1.8000   41.8000  766.4000  741.6000])

% Comarision --------------------------------------------------------------
tmp = E2H * [Camera_EEF; ones(1, N)];
R_tmp = Robot_EEF - tmp(1 : 3, :);

tmp = H2E * [Robot_EEF; ones(1, N)];
C_tmp = Camera_EEF - tmp(1 : 3, :);

StrDisp('Norm Error')
DataDisp({norm(R_tmp(1, :)), norm(R_tmp(2, :)), norm(R_tmp(3, :)), ...
          norm(C_tmp(1, :)), norm(C_tmp(2, :)), norm(C_tmp(3, :))}, ...
         {'R_X', 'R_Y', 'R_Z', 'C_X', 'C_Y', 'C_Z'})
     
disp(sum(R_tmp.^2));

StrDisp('RMSE')
DataDisp({sqrt(mean(R_tmp(1, :).^2)), sqrt(mean(R_tmp(2, :).^2)), sqrt(mean(R_tmp(3, :).^2)), ...
    sqrt(mean(C_tmp(1, :).^2)), sqrt(mean(C_tmp(2, :).^2)), sqrt(mean(C_tmp(3, :).^2))}, ...
    {'R_X', 'R_Y', 'R_Z', 'C_X', 'C_Y', 'C_Z'})


%%  Save Data =====================================================
Generate_txt('..\Result\Hand2Eye.txt', H2E, ',')
Generate_txt('..\Result\Eye2Hand.txt', E2H, ',')
     
%% Function ===============================================================
function ShowTransFormation(TransformMatrix, name)
    StrDisp([name, ' Rotation'])
    disp(TransformMatrix(1:3, 1:3))
    StrDisp([name, ' Translation'])
    disp(TransformMatrix(1:3, 4))
end

function DisplayData(Data, FittingData, StrDispname, name1, name2)
StrDisp([StrDispname, ' Data'])
DataDisp({Data(1, :)', FittingData(1, :)',   Data(2, :)', FittingData(2, :)',     Data(3, :)',   FittingData(3, :)'}, ...
         {[name1, '_x'], [name2, '_x'], ...
          [name1, '_y'], [name2, '_y'], ...
          [name1, '_z'], [name2, '_z']})
end

function DataPlot(Data, FittingData, LEGEND)
plot3(Data(1, :), Data(2, :), Data(3, :), '-+', ...
      FittingData(1, :), FittingData(2, :), FittingData(3, :), '-o'); 
xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis'); 
legend(LEGEND); axis equal; grid on;
end

function StrDisp(str, varargin)
res = '>> -----------------------------------------------------------------------';
if ~isempty(varargin), res = strrep(res, '-', varargin{1}); end

res(4 : 3 + length(str) + 1) = [str, ' '];

disp(res);
end

function DataDisp(data, name)
tmp = data{2};
cols = length(name); rows = size(tmp, 1);
DISP        = cell(rows, cols + 1);
DISP(:, 1)  = num2cell(1:rows);

for i = 2 : cols + 1
    tmp = reshape(data{i - 1}, [rows, 1]);
    for j = 1 : rows
        DISP{j, i} = tmp(j, 1);
    end
end

table       = cell2table(DISP);

table.Properties.VariableNames = [{'index'}, name(:)'];
disp(table);
end

function Generate_txt(Filename, Data, sep)
    StrDisp(['Generate File as ', Filename]);
    StrDisp(['DataSize: ', num2str(size(Data, 1)), ' * ', num2str(size(Data, 2))]);
    File = fopen(Filename, 'w');
    
    for i = 1 : size(Data, 1)
        for j = 1 : size(Data, 2)
            if j ~= size(Data, 2)
                fprintf(File, '%f%s', Data(i, j), sep);
            else
                fprintf(File, '%f', Data(i, j));
            end
        end
        if i ~= size(Data, 1)
            fprintf(File, '\n');
        end
    end
    
end