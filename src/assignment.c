#include <stdlib.h>
#include <float.h>

#include "assignment.h"

/*
 * Hungarian algorithm with potentials, O(n^2 * m) for n <= m.
 * Rows are the smaller side; when rows > cols the matrix is transposed.
 */
static int solve_rows_not_more_than_cols(
    const double *cost,
    int n,
    int m,
    int transposed,
    int rows_original,
    int cols_original,
    int row_to_col[]
) {
    double *u = calloc((size_t)(n + 1), sizeof(double));
    double *v = calloc((size_t)(m + 1), sizeof(double));
    int *p = calloc((size_t)(m + 1), sizeof(int));
    int *way = calloc((size_t)(m + 1), sizeof(int));
    double *minv = malloc((size_t)(m + 1) * sizeof(double));
    char *used = malloc((size_t)(m + 1));

    if (!u || !v || !p || !way || !minv || !used) {
        free(u); free(v); free(p); free(way); free(minv); free(used);
        return -1;
    }

    for (int i = 1; i <= n; i++) {
        p[0] = i;
        int j0 = 0;

        for (int j = 0; j <= m; j++) {
            minv[j] = DBL_MAX;
            used[j] = 0;
        }

        do {
            used[j0] = 1;
            int i0 = p[j0];
            int j1 = 0;
            double delta = DBL_MAX;

            for (int j = 1; j <= m; j++) {
                if (used[j]) {
                    continue;
                }

                double c = transposed
                    ? cost[(size_t)(j - 1) * (size_t)cols_original + (size_t)(i0 - 1)]
                    : cost[(size_t)(i0 - 1) * (size_t)cols_original + (size_t)(j - 1)];

                double current = c - u[i0] - v[j];

                if (current < minv[j]) {
                    minv[j] = current;
                    way[j] = j0;
                }

                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (int j = 0; j <= m; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    for (int i = 0; i < rows_original; i++) {
        row_to_col[i] = -1;
    }

    int assigned = 0;

    for (int j = 1; j <= m; j++) {
        if (p[j] == 0) {
            continue;
        }

        int small_index = p[j] - 1;
        int large_index = j - 1;

        if (transposed) {
            /* small side = original columns, large side = original rows */
            row_to_col[large_index] = small_index;
        }
        else {
            row_to_col[small_index] = large_index;
        }

        assigned++;
    }

    (void)cols_original;

    free(u); free(v); free(p); free(way); free(minv); free(used);

    return assigned;
}

int hungarian_assign(
    const double *cost,
    int rows,
    int cols,
    int row_to_col[]
) {
    if (cost == NULL || row_to_col == NULL || rows < 0 || cols < 0) {
        return -1;
    }

    for (int i = 0; i < rows; i++) {
        row_to_col[i] = -1;
    }

    if (rows == 0 || cols == 0) {
        return 0;
    }

    if (rows <= cols) {
        return solve_rows_not_more_than_cols(
            cost, rows, cols, 0, rows, cols, row_to_col
        );
    }

    return solve_rows_not_more_than_cols(
        cost, cols, rows, 1, rows, cols, row_to_col
    );
}
