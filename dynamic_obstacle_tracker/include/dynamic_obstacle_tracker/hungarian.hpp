// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// Hungarian Algorithm (Kuhn-Munkres) for Linear Assignment Problem
// Complexity: O(N³) vs O(N!) brute-force
//
// Reference: Harold W. Kuhn, "The Hungarian Method for the assignment problem", 1955
// Implementation based on: https://github.com/mcximing/hungarian-algorithm-cpp

#pragma once

#include <algorithm>
#include <limits>
#include <vector>

namespace dynamic_obstacle_tracker
{

class HungarianAlgorithm
{
public:
  /// @brief Solve the linear assignment problem (minimize total cost)
  /// @param cost_matrix M×N cost matrix (M tracks, N detections)
  /// @param assignment Output: assignment[i] = j means track i → detection j, or -1 if unassigned
  /// @return Total minimum cost
  static double solve(
    const std::vector<std::vector<double>> & cost_matrix,
    std::vector<int> & assignment)
  {
    const int M = static_cast<int>(cost_matrix.size());
    if (M == 0) {
      assignment.clear();
      return 0.0;
    }
    const int N = static_cast<int>(cost_matrix[0].size());
    if (N == 0) {
      assignment.assign(M, -1);
      return 0.0;
    }

    // Pad to square matrix (max(M,N) × max(M,N))
    const int dim = std::max(M, N);
    std::vector<std::vector<double>> cost(dim, std::vector<double>(dim, 0.0));
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < N; ++j) {
        cost[i][j] = cost_matrix[i][j];
      }
    }

    // Step 1: Subtract row minima
    for (int i = 0; i < dim; ++i) {
      double row_min = *std::min_element(cost[i].begin(), cost[i].end());
      for (int j = 0; j < dim; ++j) {
        cost[i][j] -= row_min;
      }
    }

    // Step 2: Subtract column minima
    for (int j = 0; j < dim; ++j) {
      double col_min = std::numeric_limits<double>::max();
      for (int i = 0; i < dim; ++i) {
        col_min = std::min(col_min, cost[i][j]);
      }
      for (int i = 0; i < dim; ++i) {
        cost[i][j] -= col_min;
      }
    }

    // Marking arrays
    std::vector<int> row_cover(dim, 0);
    std::vector<int> col_cover(dim, 0);
    std::vector<std::vector<int>> mask(dim, std::vector<int>(dim, 0));
    // mask[i][j]: 0=normal, 1=starred, 2=primed

    // Step 3: Cover columns with starred zeros
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        if (cost[i][j] == 0.0 && row_cover[i] == 0 && col_cover[j] == 0) {
          mask[i][j] = 1;  // star
          row_cover[i] = 1;
          col_cover[j] = 1;
        }
      }
    }
    // Clear row covers for next step
    std::fill(row_cover.begin(), row_cover.end(), 0);

    // Main loop
    int step = 4;
    bool done = false;
    while (!done) {
      switch (step) {
        case 4:
          step = step4(cost, mask, row_cover, col_cover, dim);
          break;
        case 5:
          step = step5(cost, mask, row_cover, col_cover, dim);
          break;
        case 6:
          step = step6(cost, mask, row_cover, col_cover, dim);
          break;
        case 7:
          done = true;
          break;
      }
    }

    // Extract assignment
    assignment.assign(M, -1);
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < N; ++j) {
        if (mask[i][j] == 1) {
          assignment[i] = j;
          break;
        }
      }
    }

    // Compute total cost
    double total_cost = 0.0;
    for (int i = 0; i < M; ++i) {
      if (assignment[i] >= 0) {
        total_cost += cost_matrix[i][assignment[i]];
      }
    }
    return total_cost;
  }

private:
  // Step 4: Find uncovered zero, prime it
  static int step4(
    const std::vector<std::vector<double>> & cost,
    std::vector<std::vector<int>> & mask,
    std::vector<int> & row_cover,
    std::vector<int> & col_cover,
    int dim)
  {
    int row = -1, col = -1;
    bool done = false;

    while (!done) {
      // Find uncovered zero
      bool found = false;
      for (int i = 0; i < dim && !found; ++i) {
        for (int j = 0; j < dim && !found; ++j) {
          if (cost[i][j] == 0.0 && row_cover[i] == 0 && col_cover[j] == 0) {
            row = i;
            col = j;
            found = true;
          }
        }
      }

      if (!found) {
        done = true;
        return 6;  // No uncovered zero, go to step 6
      }

      // Prime the zero
      mask[row][col] = 2;

      // Find starred zero in the same row
      int star_col = -1;
      for (int j = 0; j < dim; ++j) {
        if (mask[row][j] == 1) {
          star_col = j;
          break;
        }
      }

      if (star_col >= 0) {
        // Cover this row, uncover the column with starred zero
        row_cover[row] = 1;
        col_cover[star_col] = 0;
      } else {
        // No starred zero in row, go to step 5
        done = true;
        return 5;
      }
    }
    return 4;
  }

  // Step 5: Construct augmenting path and update starred zeros
  static int step5(
    const std::vector<std::vector<double>> & cost,
    std::vector<std::vector<int>> & mask,
    std::vector<int> & row_cover,
    std::vector<int> & col_cover,
    int dim)
  {
    (void)cost;
    // Find primed zero (from step 4)
    int path_row = -1, path_col = -1;
    for (int i = 0; i < dim && path_row < 0; ++i) {
      for (int j = 0; j < dim && path_col < 0; ++j) {
        if (mask[i][j] == 2) {
          path_row = i;
          path_col = j;
        }
      }
    }

    std::vector<std::pair<int, int>> path;
    path.push_back({path_row, path_col});

    bool done = false;
    while (!done) {
      // Find starred zero in the same column
      int star_row = -1;
      for (int i = 0; i < dim; ++i) {
        if (mask[i][path_col] == 1) {
          star_row = i;
          break;
        }
      }

      if (star_row >= 0) {
        path.push_back({star_row, path_col});
        // Find primed zero in the same row
        for (int j = 0; j < dim; ++j) {
          if (mask[star_row][j] == 2) {
            path_col = j;
            path.push_back({star_row, path_col});
            break;
          }
        }
      } else {
        done = true;
      }
    }

    // Augment path: unstar starred zeros, star primed zeros
    for (const auto & [r, c] : path) {
      if (mask[r][c] == 1) {
        mask[r][c] = 0;
      } else if (mask[r][c] == 2) {
        mask[r][c] = 1;
      }
    }

    // Clear covers and primes
    std::fill(row_cover.begin(), row_cover.end(), 0);
    std::fill(col_cover.begin(), col_cover.end(), 0);
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        if (mask[i][j] == 2) {
          mask[i][j] = 0;
        }
      }
    }

    return 4;
  }

  // Step 6: Add value to covered rows, subtract from uncovered columns
  static int step6(
    std::vector<std::vector<double>> & cost,
    std::vector<std::vector<int>> & mask,
    std::vector<int> & row_cover,
    std::vector<int> & col_cover,
    int dim)
  {
    (void)mask;
    // Find minimum uncovered value
    double min_val = std::numeric_limits<double>::max();
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        if (row_cover[i] == 0 && col_cover[j] == 0) {
          min_val = std::min(min_val, cost[i][j]);
        }
      }
    }

    // Add to covered rows, subtract from uncovered columns
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        if (row_cover[i] == 1) {
          cost[i][j] += min_val;
        }
        if (col_cover[j] == 0) {
          cost[i][j] -= min_val;
        }
      }
    }

    return 4;
  }
};

}  // namespace dynamic_obstacle_tracker
