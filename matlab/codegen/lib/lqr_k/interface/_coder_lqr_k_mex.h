/*
 * File: _coder_lqr_k_mex.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Sep-2024 12:03:35
 */

#ifndef _CODER_LQR_K_MEX_H
#define _CODER_LQR_K_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_lqr_k_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                              const mxArray *prhs[1]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_lqr_k_mex.h
 *
 * [EOF]
 */
