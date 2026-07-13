function posreduced = NDimDouglasPeucker(ptList, tolerance)
% NDIMDOUGLASPEUCKER Simplifies a list of N-dimensional points using the
% Douglas-Peucker algorithm.
%
% INPUTS:
%   ptList    - matrix of doubles containing the list of points.
%               Rows correspond to the number of points (n),
%               columns correspond to the dimensionality of each point (m).
%   tolerance - decimal number (double) defining the maximum allowed
%               perpendicular distance for a point to be considered
%               redundant and removed.
%
% OUTPUT:
%   posreduced - reduced list of points after simplification.
%
% Author:       Juliana Manrique Cordoba
% Email:        jmanrique@umh.es
% Institution:  Universidad Miguel Hernández de Elche
% Date:         10-07-2026
% Version:      1.0
%
% CITATION:
%   This function implements the n-dimensional Douglas-Peucker reduction
%   algorithm proposed in:
%
%   Manrique-Cordoba, J.; de la Casa-Lillo, M.A.; Sabater-Navarro, J.M.
%   N-Dimensional Reduction Algorithm for Learning from Demonstration
%   Path Planning. Sensors 2025, 25(7), 2145.
%   https://doi.org/10.3390/s25072145
%
%   Patent: File No. EP25382177, filed 28 February 2025.
% -------------------------------------------------------------------------

n = size(ptList,1);  % Number of points
m = size(ptList,2);  % Dimensionality
if n <= 2
    posreduced = ptList;
return
end
% Start and end points
A = ptList(1,:);
B = ptList(n,:);
AB = B - A;          % Vector defining the line
dNode = norm(AB);    % Length of the line
% Check that the points are not identical
if dNode < 1e-10
    posreduced = A; % If A == B, no simplification is possible
return;
end
% Orthogonal distances from each point to line AB
d = zeros(n-2,1);  % Initialize distance vector
for k = 2:n-1
    P = ptList(k,:);
    AP = P - A;
% Projection of AP onto AB
    proj = dot(AP, AB) / dot(AB, AB) * AB;
    perpVec = AP - proj; % Vector to the line
    d(k-1) = norm(perpVec); % Orthogonal distance
end
% Index of the farthest point
[~, farthestIdx] = max(d);
dmax = d(farthestIdx);
if dmax > tolerance
% Split the list into two segments and recurse
    recList1 = NDimDouglasPeucker(ptList(1:farthestIdx+1,:), tolerance);
    recList2 = NDimDouglasPeucker(ptList(farthestIdx+1:end,:), tolerance);
% Concatenate avoiding duplicates
    posreduced = [recList1; recList2(2:end,:)];
else
    posreduced = [A; B]; % Only the endpoints are kept
end
end