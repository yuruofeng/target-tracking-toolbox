function root = repoRoot()
% REPOROOT  Locate the toolbox repository root from this package.

    root = fileparts(mfilename('fullpath'));
    for iLevel = 1:7
        root = fileparts(root);
    end
end
