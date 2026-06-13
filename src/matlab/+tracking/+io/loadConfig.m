function config = loadConfig(configPath)
% LOADCONFIG  Load a JSON configuration file into a MATLAB struct.
%
%   config = tracking.io.loadConfig(configPath)

    arguments
        configPath (1, :) char
    end

    if ~isfile(configPath)
        error('tracking:io:ConfigNotFound', ...
            'Configuration file not found: %s', configPath);
    end

    text = fileread(configPath);
    config = jsondecode(text);
end
