%% Crear environment Fase 2 (F2) con 14 observables
% Vector Seq (14):
%  1  Pitch (sin)
%  2  Pitch (cos)
%  3  Roll (sin)
%  4  Roll (cos)
%  5  O1sp (sin)
%  6  O1sp (cos)
%  7  err (sin) / wrapeado
%  8  err (cos) / wrapeado
%  9  O1ini (sin)
% 10  O1ini (cos)
% 11  dO1ini (rad/s)
% 12  O1 (sin)
% 13  O1 (cos)
% 14  O1_w (rad/s)

% --- Rutas ---------------------------------------------------------------
slxPath = "/Users/richards/Documents/VIU/TFM_Lazaro/Simulador_Brazo_Lazaro/TD3_Agent/Simulador_Lazaro_RL_F3.slx";
mdlDir  = fileparts(slxPath);

addpath(mdlDir);
% savepath;  % opcional
load_system(slxPath);

mdl = "Simulador_Lazaro_RL_F3";

% --- Localizar bloque RL Agent ------------------------------------------
agentBlk = mdl + "/RL_Agent_F2";

if ~bdIsLoaded(mdl)
    error("El modelo no quedó cargado: %s", mdl);
end

try
    get_param(agentBlk,'Handle');
catch
    rlBlocks = find_system(mdl, ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'MaskType','RL Agent');

    if isempty(rlBlocks)
        rlBlocks = find_system(mdl, ...
            'LookUnderMasks','all', ...
            'FollowLinks','on', ...
            'RegExp','on', ...
            'Name','RL_Agent.*');
    end

    if isempty(rlBlocks)
        error("No se encontró el bloque RL Agent en el modelo: %s", mdl);
    end

    agentBlk = string(rlBlocks{1});
end

disp("Modelo cargado: " + mdl);
disp("Bloque agente:  " + agentBlk);

% --- Specs de observación (14) ------------------------------------------
obsInfo = rlNumericSpec([14 1], ...
    'Name','obs_F2_14', ...
    'Description', "[sin(pitch); cos(pitch); sin(roll); cos(roll); " + ...
                   "sin(O1sp); cos(O1sp); sin(e_wrap); cos(e_wrap); " + ...
                   "sin(O1ini); cos(O1ini); dO1ini(rad/s); " + ...
                   "sin(O1); cos(O1); O1_w(rad/s)]");

% Límites:
% sin/cos -> [-1, 1]
% dO1ini y O1_w -> [-2.5, 2.5]
obsInfo.LowerLimit = [ ...
    -1;   ... % 1  sin(pitch)
    -1;   ... % 2  cos(pitch)
    -1;   ... % 3  sin(roll)
    -1;   ... % 4  cos(roll)
    -1;   ... % 5  sin(O1sp)
    -1;   ... % 6  cos(O1sp)
    -1;   ... % 7  sin(e_wrap)
    -1;   ... % 8  cos(e_wrap)
    -1;   ... % 9  sin(O1ini)
    -1;   ... % 10 cos(O1ini)
    -2.5; ... % 11 dO1ini(rad/s)
    -1;   ... % 12 sin(O1)
    -1;   ... % 13 cos(O1)
    -2.5  ... % 14 O1_w(rad/s)
    ];

obsInfo.UpperLimit = [ ...
     1;   ... % 1  sin(pitch)
     1;   ... % 2  cos(pitch)
     1;   ... % 3  sin(roll)
     1;   ... % 4  cos(roll)
     1;   ... % 5  sin(O1sp)
     1;   ... % 6  cos(O1sp)
     1;   ... % 7  sin(e_wrap)
     1;   ... % 8  cos(e_wrap)
     1;   ... % 9  sin(O1ini)
     1;   ... % 10 cos(O1ini)
     2.5; ... % 11 dO1ini(rad/s)
     1;   ... % 12 sin(O1)
     1;   ... % 13 cos(O1)
     2.5  ... % 14 O1_w(rad/s)
     ];

% --- Acción: torque normalizado -----------------------------------------
actInfo = rlNumericSpec([1 1], ...
    'Name','torque_cmd', ...
    'LowerLimit', -1, ...
    'UpperLimit',  1);

% --- Crear environment ---------------------------------------------------
env_F3 = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

% env_F2.ResetFcn = @(in) localResetFcn(in);

disp("Environment creado: env_F3");
disp("Observaciones: " + string(prod(obsInfo.Dimension)));
disp("Acciones:      " + string(prod(actInfo.Dimension)));