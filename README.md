# NDimDouglasPeucker

MATLAB implementation of the **N-Dimensional Douglas-Peucker algorithm** for reducing/simplifying a list of points in any number of dimensions (2D, 3D, or higher), while preserving the overall shape of the trajectory within a given tolerance.

## Description

The classic Douglas-Peucker algorithm is widely used to simplify 2D curves (e.g., GPS tracks, polylines) by removing points that do not significantly contribute to the shape of the curve. This implementation generalizes the algorithm to work with points of **arbitrary dimensionality**, making it suitable for applications such as:

- Trajectory simplification for robotics and path planning
- Learning from Demonstration (LfD) data reduction
- Simplification of multi-dimensional sensor data
- General curve/path compression in N-dimensional space

## Usage

```matlab
posreduced = NDimDouglasPeucker(ptList, tolerance);
```

### Inputs

| Parameter   | Type            | Description |
|-------------|-----------------|-------------|
| `ptList`    | `double` matrix | List of points. Rows correspond to the number of points (`n`), columns correspond to the dimensionality of each point (`m`). |
| `tolerance` | `double`        | Maximum allowed orthogonal distance between an original point and the simplified line. Points within this tolerance are considered redundant and removed. |

### Output

| Parameter    | Type  | Description |
|--------------|-------|-------------|
| `posreduced` | same type as `ptList` | Simplified/reduced list of points, preserving the original dimensionality but with fewer points. |

### Example

```matlab
% Example with a 3D trajectory
ptList = [0 0 0;
          1 0.1 0;
          2 -0.1 0;
          3 5 0;
          4 6 0;
          5 6.1 0];

tolerance = 0.5;

posreduced = NDimDouglasPeucker(ptList, tolerance);
```

## How it works

1. The algorithm connects the first and last points of the list with a straight line.
2. It computes the orthogonal distance from every intermediate point to that line.
3. If the maximum distance exceeds `tolerance`, the point list is split at that point and the algorithm is applied recursively to both segments.
4. If the maximum distance is below `tolerance`, all intermediate points are discarded, keeping only the endpoints.

This process is repeated recursively until all remaining points fall within the specified tolerance.

## Citation

This function implements the algorithm proposed in:

> Manrique-Cordoba, J.; de la Casa-Lillo, M.A.; Sabater-Navarro, J.M. *N-Dimensional Reduction Algorithm for Learning from Demonstration Path Planning*. **Sensors** 2025, 25(7), 2145. https://doi.org/10.3390/s25072145

If you use this code in your research, please cite the paper above.

> **Note:** The results and method presented in the associated paper are covered by patent application No. **EP25382177**, filed on 28 February 2025. Please check the licensing terms before commercial use.

## Author

- **Name:** Juliana Manrique Cordoba
- **Email:** jmanrique@umh.es
- **Institution:** Universidad Miguel Hernández de Elche
