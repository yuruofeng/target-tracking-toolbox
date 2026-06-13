function root = repoRoot()
% REPOROOT  Locate the toolbox repository root from this package.

    root = fileparts(mfilename('fullpath'));
    for iLevel = 1:6
        root = fileparts(root);
    end
end
