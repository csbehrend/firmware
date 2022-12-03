#include "bigM_v2_func.h"

static const float epsilon   = 1.0e-8;
int equal(float a, float b) { return fabs(a-b) < epsilon; }

// return: flag
//         -1 = default, 0 = infeasible (not mathematically), 1 = too many iterations (20)
//          2 = unbounded (no pivot row), 3 = optimal solution within yaw_err_limit
int bigM_func(float T2F_1, float T2F_2, float T2F_3, float T2F_4, float b, 
  float A_1, float A_2, float A_3, float A_4, float beq, float Aeq_1, 
  float Aeq_2, float Aeq_3, float Aeq_4, float lb_1, float lb_2, float lb_3,
  float lb_4, float ub_1, float ub_2, float ub_3, float ub_4, float* T1, float* T2,
  float* T3, float* T4, float yaw_err_limit)
{ // Tx_sign = -1 when negative, = 1 when positive

  int flag = 0; // flag for Simplex
  float yaw_err;

  Tableau tab = {11, 24, {
    {0,     -T2F_1, -T2F_2, -T2F_3, -T2F_4, 0, 0, 0, 0, 0, 0, 0, 0, 0, bM, bM, bM, bM, bM, 0, 0, 0, 0, 1, },
    {b, A_1, A_2, A_3, A_4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
    {beq, Aeq_1, Aeq_2, Aeq_3, Aeq_4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
    {lb_1,  1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, },
    {lb_2,  0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, },
    {lb_3,  0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, },
    {lb_4,  0, 0, 0, 1, 0, 0, 0,  0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, },
    {ub_1,  1, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
    {ub_2,  0, 1, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
    {ub_3,  0, 0, 1, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
    {ub_4,  0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
  }};

  for(int i = 1; i < tab.m; i++) {
    if(tab.mat[i][0] < 0){
      if(i == 2){ // for the equality constraint
        for(int j = 0; j < 5; j++) {
          tab.mat[2][j] = - tab.mat[2][j]; // just invert equation, don't touch a1
        } 
      } else if(i >= 3 && i <= 6) { // for lb
        for(int j = 0; j <= 9; j++) {
          tab.mat[i][j] = - tab.mat[i][j];
        }
        tab.mat[i][i+12] = 0;
        tab.mat[0][i+12] = 0;
      } else if(i >= 7 && i <= 10) { // for ub
        for(int j = 0; j <= 13; j++) {
          tab.mat[i][j] = - tab.mat[i][j];
        }
        tab.mat[i][i+12] = 1;
        tab.mat[0][i+12] = bM;
      }
    }
  }

  for(int j = 0; j < tab.n; j++){
    if(tab.mat[0][j] > bM/2){
      for(int i = 1; i < tab.m; i++){
        if(tab.mat[i][j] == 1){
          for(int k = 0; k < tab.n; k++){
            tab.mat[0][k] = tab.mat[0][k] - bM * tab.mat[i][k];
          }
          break;
        }
      }
    }
  }

  flag = simplex(&tab, T1, T2, T3, T4);

  yaw_err = beq - (Aeq_1 * *T1 + Aeq_2 * *T2 + Aeq_3 * *T3 + Aeq_4 * *T4); 
  if(flag == 3 && (yaw_err > yaw_err_limit || yaw_err < - yaw_err_limit)){
    flag = 0; // infeasible (not mathmatically)
  }

  return flag;
}

void pivot_on(Tableau *tab, int row, int col) {
  int i, j;
  float pivot;

  pivot = tab->mat[row][col];

  for(j=0;j<tab->n;j++)
    tab->mat[row][j] /= pivot;

  for(i=0; i<tab->m; i++) { // foreach remaining row i do
    float multiplier = tab->mat[i][col];
    if(i==row) continue;
    for(j=0; j<tab->n; j++) { // r[i] = r[i] - z * r[row];
      tab->mat[i][j] -= multiplier * tab->mat[row][j];
    }
  }
}

// Find pivot_col = most negative column in mat[0][1..n]
int find_pivot_column(Tableau *tab) {
  int j, pivot_col = 1;
  float lowest = tab->mat[0][pivot_col];
  for(j=1; j<tab->n; j++) {
    if (tab->mat[0][j] < lowest) {
      lowest = tab->mat[0][j];
      pivot_col = j;
    }
  }

  if( lowest >= 0 ) {
    return -1; // All positive columns in row[0], this is optimal.
  }
  return pivot_col;
}

int find_pivot_row(Tableau *tab, int pivot_col) {
  int i, pivot_row = 0;
  float min_ratio = -1.0F;

  for(i=1;i<tab->m;i++){
    float ratio = tab->mat[i][0] / (tab->mat[i][pivot_col] + 0.0001);

    if ( (ratio > 0.0F  && ratio < min_ratio ) || min_ratio < 0.0F ) {
      min_ratio = ratio;
      pivot_row = i;
    }
  }

  if (min_ratio == -1)
    return -1; // Unbounded.
  return pivot_row;
}

void add_slack_variables(Tableau *tab) {
  int i, j;
  for(i=1; i<tab->m; i++) {
    for(j=1; j<tab->m; j++)
      tab->mat[i][j + tab->n -1] = (i==j);
  }
  tab->n += tab->m -1;
}

void check_b_positive(Tableau *tab) {
  int i;
  for(i=1; i<tab->m; i++)
    i = i;
    issert(tab->mat[i][0] >= 0);
}

// Given a column of identity matrix, find the row containing 1.
// return -1, if the column as not from an identity matrix.
int find_basis_variable(Tableau *tab, int col) {
  int i, xi=-1;
  for(i=1; i < tab->m; i++) {
    if (equal( tab->mat[i][col],1) ) {
      if (xi == -1)
        xi=i;   // found first '1', save this row number.
      else
        return -1; // found second '1', not an identity matrix.

    } else if (!equal( tab->mat[i][col],0) ) {
      return -1; // not an identity matrix column.
    }
  }
  return xi;
}
  

int simplex(Tableau *tab, float* T1, float* T2, float* T3, float* T4) {
  int loop=0;
  int flag = -1;

  while( ++loop ) {
    int pivot_col, pivot_row;

    pivot_col = find_pivot_column(tab);
    if( pivot_col < 0 ) {
      print_optimal_vector(tab, T1, T2, T3, T4);

      flag = 3;
      return flag; // TODO: add return value of Simplex
    }

    pivot_row = find_pivot_row(tab, pivot_col);
    if (pivot_row < 0) {
      flag = 2;
      return flag;
    }

    pivot_on(tab, pivot_row, pivot_col);
    
    if(loop > 20) {
      flag = 1;
      return flag;
    }
  }
  return 0;
}

void print_optimal_vector(Tableau *tab, float* T1, float* T2, float* T3, float* T4){
  int j, xi;
  j = 1;
  xi = find_basis_variable(tab, j);
  *T1 = tab->mat[xi][0]; // actual Torque output
  j += 1;
  xi = find_basis_variable(tab, j);
  *T2 = tab->mat[xi][0]; // actual Torque output
  j += 1;
  xi = find_basis_variable(tab, j);
  *T3 = tab->mat[xi][0]; // actual Torque output
  j += 1;
  xi = find_basis_variable(tab, j);
  *T4 = tab->mat[xi][0]; // actual Torque output
}
