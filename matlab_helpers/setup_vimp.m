function vimp_root = setup_vimp()
    % Get the directory where this script lives (matlab_helpers)
    script_path = fileparts(mfilename('fullpath'));
    
    % Go up one level to VIMP root
    vimp_root = fileparts(script_path);
    
    % Add all necessary paths
    addpath(fullfile(vimp_root, 'matlab_helpers'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/gtsam_toolbox'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/error_ellipse'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/2dpR'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/2dArm'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/2dQuad'));
    addpath(fullfile(vimp_root, 'matlab_helpers/tools/WAM'));
    
    % Save as environment variable
    setenv('VIMP_ROOT', vimp_root);
end