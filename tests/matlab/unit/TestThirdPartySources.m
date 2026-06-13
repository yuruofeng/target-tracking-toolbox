classdef TestThirdPartySources < matlab.unittest.TestCase
% TESTTHIRDPARTYSOURCES  Verify third-party source attribution records.

    methods (Test)
        function testThirdPartySourceDocumentationExists(testCase)
            repoRoot = testCase.repoRoot();
            docPath = fullfile(repoRoot, 'docs', 'third-party-sources.md');

            testCase.verifyTrue(isfile(docPath));
            content = fileread(docPath);

            testCase.verifyNotEmpty(strfind(content, 'linh-gist/labeledRFS'));
            testCase.verifyNotEmpty(strfind(content, 'yuhsuansia/Extended-target-PMBM-tracker'));
            testCase.verifyNotEmpty(strfind(content, 'OmegaEta/Muti-scans-Smoothing-Multiple-Extended-Object-Tracking'));
            testCase.verifyNotEmpty(strfind(content, 'Not integrated'));
        end

        function testThirdPartyLicenseFilesExist(testCase)
            repoRoot = testCase.repoRoot();
            licenseRoot = fullfile(repoRoot, 'resources', 'third_party', 'licenses');

            expectedFiles = {
                'labeledRFS-MIT.txt'
                'extended-target-pmbm-BSD-2-Clause.txt'
                'ggiw-pmbm-smoother-BSD-2-Clause.txt'
            };

            for iFile = 1:numel(expectedFiles)
                filePath = fullfile(licenseRoot, expectedFiles{iFile});
                testCase.verifyTrue(isfile(filePath), expectedFiles{iFile});
                testCase.verifyGreaterThan(strlength(fileread(filePath)), 100);
            end
        end

        function testThirdPartyMatlabSourcesAreVendoredOutsidePath(testCase)
            repoRoot = testCase.repoRoot();
            sourceRoot = fullfile(repoRoot, 'resources', 'third_party', 'matlab');

            testCase.verifyTrue(isfolder(fullfile(sourceRoot, 'labeledRFS')));
            testCase.verifyTrue(isfolder(fullfile(sourceRoot, 'extended-target-pmbm')));
            testCase.verifyTrue(isfolder(fullfile(sourceRoot, 'ggiw-pmbm-smoother')));
        end
    end

    methods
        function root = repoRoot(~)
            root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
        end
    end
end
