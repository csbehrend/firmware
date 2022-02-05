#ifndef _SIMPLEX_STEP_300_ 
#define _SIMPLEX_STEP_300_

typedef struct {
  int m, n; // m=rows, n=columns, mat[m x n]
  double mat[20][20];
} Tableau;

void simplex(Tableau *tab);

#endif