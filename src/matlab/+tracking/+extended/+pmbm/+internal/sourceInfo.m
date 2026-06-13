function info = sourceInfo(sourceName)
% SOURCEINFO  Return third-party source metadata for extended PMBM adapters.

    if nargin < 1 || isempty(sourceName)
        sourceName = 'extended-target-pmbm';
    end

    repoRoot = tracking.extended.pmbm.internal.repoRoot();
    sourceRoot = fullfile(repoRoot, 'resources', 'third_party', 'matlab', sourceName);

    switch sourceName
        case 'ggiw-pmbm-smoother'
            repository = 'https://github.com/OmegaEta/Muti-scans-Smoothing-Multiple-Extended-Object-Tracking';
        otherwise
            repository = 'https://github.com/yuhsuansia/Extended-target-PMBM-tracker';
    end

    info = struct( ...
        'name', sourceName, ...
        'repository', repository, ...
        'license', 'BSD-2-Clause', ...
        'sourceRoot', sourceRoot);
end
