function varargout = demo_extended_starconvex()
% DEMO_EXTENDED_STARCONVEX  Run the star-convex extended-target adapter demo.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'extended', 'starconvex_default.json'));
    tracker = tracking.extended.starconvex.StarConvexTracker(config);
    result = tracker.run(config);

    outputFile = fullfile(resultsDir, 'demo_extended_starconvex.mat');
    save(outputFile, 'result');
    fprintf('Saved star-convex adapter demo result to %s\n', outputFile);

    if nargout > 0
        varargout{1} = result;
    end
end
