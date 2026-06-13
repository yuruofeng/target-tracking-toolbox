classdef TestPackageStructure < matlab.unittest.TestCase
% TESTPACKAGESTRUCTURE  Verify the public tracking namespace layout.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testCorePackagesExist(testCase)
            testCase.verifyEqual(exist('tracking.single.dbt.EKF', 'class'), 8);
            testCase.verifyEqual(exist('tracking.single.tbd.DpTbd', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.phd.ImmPhdFilter', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.ggiw.GgiwFilter', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.starconvex.StarConvexTracker', 'class'), 8);
        end

        function testSharedPackagesExist(testCase)
            testCase.verifyEqual(exist('tracking.association.Hungarian', 'class'), 8);
            testCase.verifyEqual(exist('tracking.metrics.OspaMetric', 'class'), 8);
            testCase.verifyEqual(exist('tracking.models.MeasurementModel', 'class'), 8);
            testCase.verifyNotEmpty(which('tracking.io.loadConfig'));
            testCase.verifyEqual(exist('tracking.learning.PythonModelAdapter', 'class'), 8);
        end
    end
end
