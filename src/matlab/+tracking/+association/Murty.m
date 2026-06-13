classdef Murty < handle
% MURTY  K-best assignment solver for small association problems.

    properties
        NumBest (1,1) double = 1
        Assignments (:,:) double = []
        Costs (:,1) double = []
        History struct = struct()
    end

    methods
        function obj = Murty(kBest)
            if nargin > 0
                obj.NumBest = kBest;
            end
        end

        function [assignments, costs] = solve(obj, costMatrix)
            if ~ismatrix(costMatrix) || size(costMatrix, 1) ~= size(costMatrix, 2)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                    'Cost matrix must be square.');
                throw(ME);
            end

            n = size(costMatrix, 1);
            if n > 8
                ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                    'Murty exhaustive fallback supports at most 8 targets, got %d.', n);
                throw(ME);
            end

            allAssignments = perms(1:n);
            allCosts = zeros(size(allAssignments, 1), 1);
            for iAssignment = 1:size(allAssignments, 1)
                linearIdx = sub2ind([n, n], 1:n, allAssignments(iAssignment, :));
                allCosts(iAssignment) = sum(costMatrix(linearIdx));
            end

            validIdx = isfinite(allCosts);
            allAssignments = allAssignments(validIdx, :);
            allCosts = allCosts(validIdx);

            [costs, order] = sort(allCosts, 'ascend');
            allAssignments = allAssignments(order, :);

            count = min(obj.NumBest, numel(costs));
            assignments = allAssignments(1:count, :);
            costs = costs(1:count);

            obj.Assignments = assignments;
            obj.Costs = costs;
            obj.History = struct( ...
                'numBest', obj.NumBest, ...
                'totalFound', count, ...
                'assignments', obj.Assignments, ...
                'costs', obj.Costs);
        end

        function displayResults(obj)
            if isempty(obj.Assignments)
                fprintf('No Murty assignments available.\n');
                return;
            end

            fprintf('\n===== Murty assignment results =====\n');
            fprintf('Requested: %d | Found: %d\n', obj.NumBest, size(obj.Assignments, 1));
            for iAssignment = 1:size(obj.Assignments, 1)
                fprintf('%d: assignment=%s cost=%.4f\n', ...
                    iAssignment, mat2str(obj.Assignments(iAssignment, :)), obj.Costs(iAssignment));
            end
        end
    end

    methods (Static)
        function [assignments, costs] = run(costMatrix, kBest)
            solver = tracking.association.Murty(kBest);
            [assignments, costs] = solver.solve(costMatrix);
        end
    end
end
