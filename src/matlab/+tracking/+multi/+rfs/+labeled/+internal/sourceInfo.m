function info = sourceInfo(sourceName)
% SOURCEINFO  Return third-party source metadata for labeled RFS adapters.

    if nargin < 1 || isempty(sourceName)
        sourceName = 'labeledRFS';
    end

    repoRoot = tracking.multi.rfs.labeled.internal.repoRoot();
    sourceRoot = fullfile(repoRoot, 'resources', 'third_party', 'matlab', sourceName);

    info = struct( ...
        'name', sourceName, ...
        'repository', 'https://github.com/linh-gist/labeledRFS', ...
        'license', 'MIT', ...
        'sourceRoot', sourceRoot);
end
