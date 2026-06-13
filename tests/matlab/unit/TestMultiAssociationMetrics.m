classdef TestMultiAssociationMetrics < matlab.unittest.TestCase
% TESTMULTIASSOCIATIONMETRICS  Validate migrated association and metrics code.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testAssignmentAlgorithmsReturnValidSolutions(testCase)
            costMatrix = [10 2 3; 4 7 1; 5 8 6];

            murty = tracking.association.Murty(2);
            [assignments, costs] = murty.solve(costMatrix);
            testCase.verifySize(assignments, [2 3]);
            testCase.verifyNumElements(costs, 2);

            auction = tracking.association.Auction();
            [personToObject, objectToPerson, auctionCost] = auction.solve(costMatrix);
            testCase.verifyNumElements(personToObject, 3);
            testCase.verifyNumElements(objectToPerson, 3);
            testCase.verifyTrue(isnumeric(auctionCost));

            munkres = tracking.association.Munkres();
            [munkresAssignment, munkresCost] = munkres.solve(costMatrix);
            testCase.verifyNumElements(munkresAssignment, 3);
            testCase.verifyTrue(isnumeric(munkresCost));
        end

        function testAssignmentFactoryDoesNotShadowPackageName(testCase)
            algorithm = tracking.association.AssignmentFactory.createAlgorithm('Murty', 1);
            testCase.verifyClass(algorithm, 'tracking.association.Murty');

            [assignmentResult, assignmentCost] = ...
                tracking.association.AssignmentFactory.solve([1 2; 3 4], 'Auction');
            testCase.verifyNumElements(assignmentResult, 2);
            testCase.verifyTrue(isnumeric(assignmentCost));
        end

        function testGospaHandlesEmptyAndNonEmptySets(testCase)
            gospa = tracking.metrics.GOSPA('p', 2, 'c', 10, 'alpha', 2);

            distance = gospa.compute([], []);
            testCase.verifyEqual(distance, 0);

            x = [1 2 3; 4 5 6];
            y = [1.1 2.2 3.3; 4.4 5.5 6.6];
            [distance, assignmentResult, decomposition] = gospa.compute(x, y);
            testCase.verifyGreaterThanOrEqual(distance, 0);
            testCase.verifyNumElements(assignmentResult, 3);
            testCase.verifyTrue(isfield(decomposition, 'localisation'));
            testCase.verifyTrue(isfield(decomposition, 'missed'));
            testCase.verifyTrue(isfield(decomposition, 'false'));
        end

        function testGospaRejectsMismatchedDimensions(testCase)
            gospa = tracking.metrics.GOSPA();
            testCase.verifyError(@() gospa.compute(zeros(2, 1), zeros(3, 1)), ...
                'MTT:Error1004');
        end
    end
end
