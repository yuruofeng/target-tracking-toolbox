function varargout = demo_all(varargin)
% DEMO_ALL  Complete target tracking algorithm demonstration.
%   Runs both DBT and TBD algorithm demonstrations with comprehensive
%   visualization and performance comparison.
%
%   Usage:
%       demo_all()                    % Run both demos with defaults
%       demo_all('mode', 'dbt')       % Run only DBT demo
%       demo_all('mode', 'tbd')       % Run only TBD demo
%       demo_all('mcRuns', 5)         % 5 MC runs for DBT
%
%   Parameters (name-value pairs):
%       mode        - 'both', 'dbt', or 'tbd' (default: 'both')
%       mcRuns      - MC runs for DBT (default: 1)
%       saveResults - Save results to files (default: false)
%       showPlots   - Display plots (default: true)
%
%   Outputs:
%       results - Struct containing DBT and TBD results
%
%   Example:
%       results = demo_all('mode', 'both', 'mcRuns', 3);
%
%   See also: demo_single_dbt, demo_single_tbd

    close all; clc;
    exampleFolder = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(exampleFolder));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));

    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('    Target Tracking Algorithm Suite - Complete Demonstration\n');
    fprintf('================================================================\n\n');

    p = inputParser;
    addParameter(p, 'mode', 'both', @(x) ismember(lower(x), {'both', 'dbt', 'tbd'}));
    addParameter(p, 'mcRuns', 1, @isscalar);
    addParameter(p, 'saveResults', false, @islogical);
    addParameter(p, 'showPlots', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    results = struct();
    mode = lower(opts.mode);

    if ismember(mode, {'both', 'dbt'})
        fprintf('\n>>> Running DBT Demo <<<\n');
        dbtResults = demo_single_dbt('mcRuns', opts.mcRuns, ...
                            'saveResults', opts.saveResults, ...
                            'saveFile', fullfile(repoRoot, 'results', 'dbt_results.mat'), ...
                            'showPlots', opts.showPlots);
        results.dbt = dbtResults;
    end

    if ismember(mode, {'both', 'tbd'})
        fprintf('\n>>> Running TBD Demo <<<\n');
        tbdResults = demo_single_tbd('saveResults', opts.saveResults, ...
                            'saveFile', fullfile(repoRoot, 'results', 'tbd_results.mat'), ...
                            'showPlots', opts.showPlots);
        results.tbd = tbdResults;
    end

    fprintf('\n================================================================\n');
    fprintf('    All Demonstrations Complete\n');
    fprintf('================================================================\n\n');

    if nargout > 0
        varargout{1} = results;
    end
end
