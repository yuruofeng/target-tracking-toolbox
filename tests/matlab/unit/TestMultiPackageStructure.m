classdef TestMultiPackageStructure < matlab.unittest.TestCase
% TESTMULTIPACKAGESTRUCTURE  Verify imported multi-target namespaces.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testSharedMultiTargetPackagesExist(testCase)
            testCase.verifyEqual(exist('tracking.association.Murty', 'class'), 8);
            testCase.verifyEqual(exist('tracking.association.Auction', 'class'), 8);
            testCase.verifyEqual(exist('tracking.association.Munkres', 'class'), 8);
            testCase.verifyEqual(exist('tracking.association.AssignmentFactory', 'class'), 8);

            testCase.verifyEqual(exist('tracking.metrics.GOSPA', 'class'), 8);
            testCase.verifyEqual(exist('tracking.metrics.TrajectoryMetric', 'class'), 8);
            testCase.verifyEqual(exist('tracking.metrics.TrajectoryErrorCalculator', 'class'), 8);

            testCase.verifyEqual(exist('tracking.core.ErrorCode', 'class'), 8);
            testCase.verifyEqual(exist('tracking.core.MTTException', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.core.FilterConfig', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.core.FilterResult', 'class'), 8);
        end

        function testRfsFilterPackagesExist(testCase)
            testCase.verifyEqual(exist('tracking.multi.rfs.phd.GMPHD', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.phd.GMCPHD', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.phd.PHDFactory', 'class'), 8);

            testCase.verifyEqual(exist('tracking.multi.rfs.pmbm.PMB', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.pmbm.PMBM', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.pmbm.PMBMFactory', 'class'), 8);

            testCase.verifyEqual(exist('tracking.multi.rfs.cd.CDGMPHD', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.cd.CDGMCPHD', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.cd.CDPMBM', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.cd.CDFilterFactory', 'class'), 8);
        end

        function testTrajectoryFilterPackagesExist(testCase)
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.phd.GMTPHD', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.phd.TPHDFactory', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.pmb.TPMB', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.pmbm.TPMBM', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.pmbm.TPMBMFactory', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.pmbm.TPMBFactory', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.trajectory.mbm.TMBM', 'class'), 8);
        end
    end
end
