#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

/*
 * Optimal one-to-one assignment (Hungarian / Kuhn-Munkres algorithm).
 *
 * cost        : rows x cols matrix in row-major order
 * row_to_col  : output array of size rows; the column assigned to each row,
 *               or -1 when the row is not assigned
 *
 * The matrix may be rectangular. The total cost of the assignment is
 * minimised. Callers that want to forbid a pair should give it a very
 * large cost and check the cost of every returned pair afterwards.
 *
 * Returns the number of rows that received a column, or -1 on failure.
 */
int hungarian_assign(
    const double *cost,
    int rows,
    int cols,
    int row_to_col[]
);

#endif
