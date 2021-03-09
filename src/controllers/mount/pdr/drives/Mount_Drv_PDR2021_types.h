/*
 * File: Mount_Drv_PDR2021_types.h
 *
 * Code generated for Simulink model 'Mount_Drv_PDR2021'.
 *
 * Model version                  : 1.952
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Tue Mar  2 10:16:45 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Mount_Drv_PDR2021_types_h_
#define RTW_HEADER_Mount_Drv_PDR2021_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_1utrxinkom0IYpuKJbdriH_
#define DEFINED_TYPEDEF_FOR_struct_1utrxinkom0IYpuKJbdriH_

typedef struct {
  real_T sigmv;
  real_T iIsSig01Active;
  real_T sigm0;
  real_T sigm1;
  real_T Tc;
  real_T Ts;
  real_T vs;
  real_T iIsActive;
} struct_1utrxinkom0IYpuKJbdriH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8qO4tAG9pSAbUXCq20dmND_
#define DEFINED_TYPEDEF_FOR_struct_8qO4tAG9pSAbUXCq20dmND_

typedef struct {
  real_T sigmv;
  real_T iIsSig01Active;
  real_T Tc;
  real_T Ts;
  real_T vs;
  real_T sigm0;
  real_T zfr;
  real_T sigm1;
  real_T iIsActive;
} struct_8qO4tAG9pSAbUXCq20dmND;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_1tG6fUrrHXwmOHi4Wk0jIF_
#define DEFINED_TYPEDEF_FOR_struct_1tG6fUrrHXwmOHi4Wk0jIF_

typedef struct {
  real_T Amp[361];
} struct_1tG6fUrrHXwmOHi4Wk0jIF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_chGxBiYsRNaLF2MmBzpyTG_
#define DEFINED_TYPEDEF_FOR_struct_chGxBiYsRNaLF2MmBzpyTG_

typedef struct {
  real_T Pos[361];
  real_T R;
  real_T N_Segm;
  struct_1tG6fUrrHXwmOHi4Wk0jIF oCog;
  struct_1tG6fUrrHXwmOHi4Wk0jIF oEmf;
} struct_chGxBiYsRNaLF2MmBzpyTG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_Io4x7ySdlOLKH9IQzqvteF_
#define DEFINED_TYPEDEF_FOR_struct_Io4x7ySdlOLKH9IQzqvteF_

typedef struct {
  real_T sfr;
  real_T amp;
} struct_Io4x7ySdlOLKH9IQzqvteF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xNUiBaTweEDVaLcXKH5F9_
#define DEFINED_TYPEDEF_FOR_struct_xNUiBaTweEDVaLcXKH5F9_

typedef struct {
  real_T sfr;
  real_T ampFact;
} struct_xNUiBaTweEDVaLcXKH5F9;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_PY3369GGOBCAR03akSzcTE_
#define DEFINED_TYPEDEF_FOR_struct_PY3369GGOBCAR03akSzcTE_

typedef struct {
  struct_Io4x7ySdlOLKH9IQzqvteF oCog;
  struct_xNUiBaTweEDVaLcXKH5F9 oEmf;
} struct_PY3369GGOBCAR03akSzcTE;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_Mount_Drv_PDR2021_T RT_MODEL_Mount_Drv_PDR2021_T;

#endif                                 /* RTW_HEADER_Mount_Drv_PDR2021_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
