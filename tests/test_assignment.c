#include <stdio.h>
#include <math.h>
#include "assignment.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

static double total(const double *c, int cols, const int *r2c, int rows) {
    double t = 0; for (int i = 0; i < rows; i++) if (r2c[i] >= 0) t += c[i * cols + r2c[i]]; return t;
}

int main(void) {
    /* square: optimum is 1+2+3 = 6 (greedy picking row-wise minima would give more) */
    double a[9] = {1, 2, 3,  2, 4, 6,  3, 6, 9};
    int r[3];
    CHECK(hungarian_assign(a, 3, 3, r) == 3, "square assigns 3");
    /* classic case where greedy fails: greedy takes (0,0)=1 then (1,1)=100; optimum is 2+2 */
    double g[4] = {1, 2,  2, 100};
    CHECK(hungarian_assign(g, 2, 2, r) == 2, "greedy-trap assigns 2");
    CHECK(fabs(total(g, 2, r, 2) - 4.0) < 1e-9, "greedy-trap optimal cost 4");
    /* more rows than columns */
    double t[6] = {5, 1,  1, 5,  3, 3};
    int r3[3];
    int n = hungarian_assign(t, 3, 2, r3);
    CHECK(n == 2, "tall matrix assigns min(rows,cols)");
    CHECK(fabs(total(t, 2, r3, 3) - 2.0) < 1e-9, "tall matrix optimal cost 2");
    /* more columns than rows */
    double w[6] = {9, 1, 9,  9, 9, 1};
    int r2[2];
    CHECK(hungarian_assign(w, 2, 3, r2) == 2, "wide assigns 2");
    CHECK(r2[0] == 1 && r2[1] == 2, "wide picks columns 1 and 2");
    /* one-to-one: no column used twice */
    double b[16]; for (int i = 0; i < 16; i++) b[i] = (i * 7) % 5;
    int r4[4]; hungarian_assign(b, 4, 4, r4);
    for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) CHECK(r4[i] != r4[j], "columns unique");
    printf(failures ? "assignment tests: %d FAILED\n" : "assignment tests: all passed\n", failures);
    return failures ? 1 : 0;
}
