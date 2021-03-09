/*
 * File: Mount_Control.c
 *
 * Code generated for Simulink model 'Mount_Control'.
 *
 * Model version                  : 1.956
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Mon Mar  8 16:47:21 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Mount_Control.h"
#include "Mount_Control_private.h"

/* Block signals (default storage) */
B_Mount_Control_T Mount_Control_B;

/* Block states (default storage) */
DW_Mount_Control_T Mount_Control_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Mount_Control_T Mount_Control_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Mount_Control_T Mount_Control_Y;

/* Real-time model */
RT_MODEL_Mount_Control_T Mount_Control_M_;
RT_MODEL_Mount_Control_T *const Mount_Control_M = &Mount_Control_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Mount_Control_M->Timing.TaskCounters.TID[1])++;
  if ((Mount_Control_M->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.005s, 0.0s] */
    Mount_Control_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
void Mount_Control_step(void)
{
  /* local block i/o variables */
  real_T rtb_DiscreteSS;
  real_T rtb_DiscreteSS_p;
  real_T rtb_DiscreteSS_n;
  real_T rtb_DiscreteSS_np;
  real_T rtb_DiscreteSS_pk;
  real_T rtb_DiscreteSS_f;
  real_T rtb_Sum1;
  real_T rtb_az_ff_en;
  real_T rtb_Sum;
  int32_T i;

  /* RateTransition: '<S3>/RTout' incorporates:
   *  RateTransition: '<S5>/RTout'
   */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTout = Mount_Control_DW.RTout_Buffer0;

    /* RateTransition: '<S2>/RTout' */
    Mount_Control_B.RTout_g = Mount_Control_DW.RTout_Buffer0_b;
    Mount_Control_B.RTout_j = Mount_Control_DW.RTout_Buffer0_k;

    /* RateTransition: '<S4>/RTout' */
    Mount_Control_B.RTout_k = Mount_Control_DW.RTout_Buffer0_m;
  }

  /* End of RateTransition: '<S3>/RTout' */

  /* Sum: '<S1>/Sum2' */
  rtb_Sum = Mount_Control_B.RTout + Mount_Control_B.RTout_g;

  /* Sum: '<S1>/Sum3' */
  rtb_az_ff_en = Mount_Control_B.RTout_j + Mount_Control_B.RTout_k;

  /* RateTransition: '<S7>/RTout' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTout_c = Mount_Control_DW.RTout_Buffer0_mf;

    /* RateTransition: '<S6>/RTout' */
    Mount_Control_B.RTout_m = Mount_Control_DW.RTout_Buffer0_bt;
  }

  /* End of RateTransition: '<S7>/RTout' */

  /* Sum: '<S1>/Sum5' */
  rtb_Sum1 = Mount_Control_B.RTout_c + Mount_Control_B.RTout_m;

  /* Outport: '<Root>/Mount_cmd' */
  Mount_Control_Y.Mount_cmd[0] = rtb_Sum;
  Mount_Control_Y.Mount_cmd[1] = rtb_az_ff_en;
  Mount_Control_Y.Mount_cmd[2] = rtb_Sum1;

  /* Gain: '<S1>/AZ average and  m2rad conv+' incorporates:
   *  Inport: '<Root>/Mount_FB'
   */
  rtb_Sum1 = ((0.25 * Mount_Control_U.Mount_FB[0] + 0.25 *
               Mount_Control_U.Mount_FB[1]) + 0.25 * Mount_Control_U.Mount_FB[2])
    + 0.25 * Mount_Control_U.Mount_FB[3];

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_Sum1 = Mount_Control_U.Mount_SP[0] - rtb_Sum1;

  /* RateTransition: '<S2>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin = rtb_Sum1;

    /* DiscreteStateSpace: '<S2>/Discrete SS' */
    {
      {
        static const int_T colCidxRow0[11] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
        };

        const int_T *pCidx = &colCidxRow0[0];
        const real_T *pC0 = Mount_Control_ConstP.DiscreteSS_C;
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *y0 = &rtb_DiscreteSS;
        int_T numNonZero = 10;
        *y0 = (*pC0++) * xd[*pCidx++];
        while (numNonZero--) {
          *y0 += (*pC0++) * xd[*pCidx++];
        }
      }

      rtb_DiscreteSS += 9.6845239959020271E+9*Mount_Control_B.RTin;
    }
  }

  /* End of RateTransition: '<S2>/RTin' */

  /* Gain: '<S1>/az_ff_en' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_az_ff_en = 0.0 * Mount_Control_U.Mount_SP[0];

  /* RateTransition: '<S3>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin_k = rtb_az_ff_en;

    /* DiscreteStateSpace: '<S3>/Discrete SS' */
    {
      rtb_DiscreteSS_p = (725703.23025879019)*
        Mount_Control_DW.DiscreteSS_DSTATE_i[0]
        + (-124510.98979647842)*Mount_Control_DW.DiscreteSS_DSTATE_i[1];
      rtb_DiscreteSS_p += 1.9099163395092613E+10*Mount_Control_B.RTin_k;
    }
  }

  /* End of RateTransition: '<S3>/RTin' */

  /* Gain: '<S1>/EL average and  m2rad conv+' incorporates:
   *  Inport: '<Root>/Mount_FB'
   */
  rtb_Sum = 0.0;
  for (i = 0; i < 6; i++) {
    rtb_Sum += Mount_Control_U.Mount_FB[i + 4] * 0.16666666666666666;
  }

  /* End of Gain: '<S1>/EL average and  m2rad conv+' */

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_Sum = Mount_Control_U.Mount_SP[1] - rtb_Sum;

  /* RateTransition: '<S4>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin_g = rtb_Sum;

    /* DiscreteStateSpace: '<S4>/Discrete SS' */
    {
      {
        static const int_T colCidxRow0[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
          11, 12 };

        const int_T *pCidx = &colCidxRow0[0];
        const real_T *pC0 = Mount_Control_ConstP.DiscreteSS_C_o;
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *y0 = &rtb_DiscreteSS_n;
        int_T numNonZero = 12;
        *y0 = (*pC0++) * xd[*pCidx++];
        while (numNonZero--) {
          *y0 += (*pC0++) * xd[*pCidx++];
        }
      }

      rtb_DiscreteSS_n += 1.1659108338015377E+10*Mount_Control_B.RTin_g;
    }
  }

  /* End of RateTransition: '<S4>/RTin' */

  /* Gain: '<S1>/el_ff_en' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_Sum = 0.0 * Mount_Control_U.Mount_SP[1];

  /* RateTransition: '<S5>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin_o = rtb_Sum;

    /* DiscreteStateSpace: '<S5>/Discrete SS' */
    {
      rtb_DiscreteSS_np = (640680.60984232358)*
        Mount_Control_DW.DiscreteSS_DSTATE_b[0]
        + (-109923.4143500115)*Mount_Control_DW.DiscreteSS_DSTATE_b[1];
      rtb_DiscreteSS_np += 1.4886047221052851E+10*Mount_Control_B.RTin_o;
    }
  }

  /* End of RateTransition: '<S5>/RTin' */

  /* Gain: '<S1>/GIR average and  m2rad conv+' incorporates:
   *  Inport: '<Root>/Mount_FB'
   */
  rtb_Sum = ((0.25 * Mount_Control_U.Mount_FB[10] + 0.25 *
              Mount_Control_U.Mount_FB[11]) + 0.25 * Mount_Control_U.Mount_FB[12])
    + 0.25 * Mount_Control_U.Mount_FB[13];

  /* Sum: '<S1>/Sum4' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_Sum = Mount_Control_U.Mount_SP[2] - rtb_Sum;

  /* RateTransition: '<S6>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin_b = rtb_Sum;

    /* DiscreteStateSpace: '<S6>/Discrete SS' */
    {
      {
        static const int_T colCidxRow0[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pCidx = &colCidxRow0[0];
        const real_T *pC0 = Mount_Control_ConstP.DiscreteSS_C_or;
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *y0 = &rtb_DiscreteSS_pk;
        int_T numNonZero = 7;
        *y0 = (*pC0++) * xd[*pCidx++];
        while (numNonZero--) {
          *y0 += (*pC0++) * xd[*pCidx++];
        }
      }

      rtb_DiscreteSS_pk += 3.2431128644822211E+9*Mount_Control_B.RTin_b;
    }
  }

  /* End of RateTransition: '<S6>/RTin' */

  /* Gain: '<S1>/gir_ff_en' incorporates:
   *  Inport: '<Root>/Mount_SP'
   */
  rtb_Sum = 0.0 * Mount_Control_U.Mount_SP[2];

  /* RateTransition: '<S7>/RTin' */
  if (Mount_Control_M->Timing.TaskCounters.TID[1] == 0) {
    Mount_Control_B.RTin_c = rtb_Sum;

    /* DiscreteStateSpace: '<S7>/Discrete SS' */
    {
      rtb_DiscreteSS_f = (91226.608932439645)*
        Mount_Control_DW.DiscreteSS_DSTATE_h[0]
        + (-15652.01159419358)*Mount_Control_DW.DiscreteSS_DSTATE_h[1];
      rtb_DiscreteSS_f += 3.0181394007059932E+8*Mount_Control_B.RTin_c;
    }

    /* Update for RateTransition: '<S3>/RTout' */
    Mount_Control_DW.RTout_Buffer0 = rtb_DiscreteSS_p;

    /* Update for RateTransition: '<S2>/RTout' */
    Mount_Control_DW.RTout_Buffer0_b = rtb_DiscreteSS;

    /* Update for RateTransition: '<S5>/RTout' */
    Mount_Control_DW.RTout_Buffer0_k = rtb_DiscreteSS_np;

    /* Update for RateTransition: '<S4>/RTout' */
    Mount_Control_DW.RTout_Buffer0_m = rtb_DiscreteSS_n;

    /* Update for RateTransition: '<S7>/RTout' */
    Mount_Control_DW.RTout_Buffer0_mf = rtb_DiscreteSS_f;

    /* Update for RateTransition: '<S6>/RTout' */
    Mount_Control_DW.RTout_Buffer0_bt = rtb_DiscreteSS_pk;

    /* Update for DiscreteStateSpace: '<S2>/Discrete SS' */
    {
      real_T xnew[11];
      xnew[0] = (1.0)*Mount_Control_DW.DiscreteSS_DSTATE[0];
      xnew[0] += (-77662.685036233073)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow1[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow1[0];
        const real_T *pA1 = &Mount_Control_ConstP.DiscreteSS_A[1];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew1 = &xnew[1];
        int_T numNonZero = 9;
        *pxnew1 = (*pA1++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew1 += (*pA1++) * xd[*pAidx++];
        }
      }

      xnew[1] += (7537.1284540506576)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow2[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow2[0];
        const real_T *pA11 = &Mount_Control_ConstP.DiscreteSS_A[11];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew2 = &xnew[2];
        int_T numNonZero = 9;
        *pxnew2 = (*pA11++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew2 += (*pA11++) * xd[*pAidx++];
        }
      }

      xnew[2] += (2762.2374752305582)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow3[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow3[0];
        const real_T *pA21 = &Mount_Control_ConstP.DiscreteSS_A[21];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew3 = &xnew[3];
        int_T numNonZero = 9;
        *pxnew3 = (*pA21++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew3 += (*pA21++) * xd[*pAidx++];
        }
      }

      xnew[3] += (2852.4474041264766)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow4[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow4[0];
        const real_T *pA31 = &Mount_Control_ConstP.DiscreteSS_A[31];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew4 = &xnew[4];
        int_T numNonZero = 9;
        *pxnew4 = (*pA31++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew4 += (*pA31++) * xd[*pAidx++];
        }
      }

      xnew[4] += (-580.22123691239062)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow5[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow5[0];
        const real_T *pA41 = &Mount_Control_ConstP.DiscreteSS_A[41];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew5 = &xnew[5];
        int_T numNonZero = 9;
        *pxnew5 = (*pA41++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew5 += (*pA41++) * xd[*pAidx++];
        }
      }

      xnew[5] += (1239.7114854959368)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow6[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow6[0];
        const real_T *pA51 = &Mount_Control_ConstP.DiscreteSS_A[51];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew6 = &xnew[6];
        int_T numNonZero = 9;
        *pxnew6 = (*pA51++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew6 += (*pA51++) * xd[*pAidx++];
        }
      }

      xnew[6] += (1439.8165041222087)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow7[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow7[0];
        const real_T *pA61 = &Mount_Control_ConstP.DiscreteSS_A[61];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew7 = &xnew[7];
        int_T numNonZero = 9;
        *pxnew7 = (*pA61++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew7 += (*pA61++) * xd[*pAidx++];
        }
      }

      xnew[7] += (3268.7361695673153)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow8[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow8[0];
        const real_T *pA71 = &Mount_Control_ConstP.DiscreteSS_A[71];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew8 = &xnew[8];
        int_T numNonZero = 9;
        *pxnew8 = (*pA71++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew8 += (*pA71++) * xd[*pAidx++];
        }
      }

      xnew[8] += (559.445217753392)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow9[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow9[0];
        const real_T *pA81 = &Mount_Control_ConstP.DiscreteSS_A[81];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew9 = &xnew[9];
        int_T numNonZero = 9;
        *pxnew9 = (*pA81++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew9 += (*pA81++) * xd[*pAidx++];
        }
      }

      xnew[9] += (14.753959523287129)*Mount_Control_B.RTin;

      {
        static const int_T colAidxRow10[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

        const int_T *pAidx = &colAidxRow10[0];
        const real_T *pA91 = &Mount_Control_ConstP.DiscreteSS_A[91];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE[0];
        real_T *pxnew10 = &xnew[10];
        int_T numNonZero = 9;
        *pxnew10 = (*pA91++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew10 += (*pA91++) * xd[*pAidx++];
        }
      }

      xnew[10] += (-120.63299636638453)*Mount_Control_B.RTin;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE[0], xnew,
                    sizeof(real_T)*11);
    }

    /* Update for DiscreteStateSpace: '<S3>/Discrete SS' */
    {
      real_T xnew[2];
      xnew[0] = (0.89737819299898691)*Mount_Control_DW.DiscreteSS_DSTATE_i[0]
        + (0.04172317442530582)*Mount_Control_DW.DiscreteSS_DSTATE_i[1];
      xnew[0] += (-3284.5198766745971)*Mount_Control_B.RTin_k;
      xnew[1] = (-0.041723174425305792)*Mount_Control_DW.DiscreteSS_DSTATE_i[0]
        + (0.98082454184959844)*Mount_Control_DW.DiscreteSS_DSTATE_i[1];
      xnew[1] += (-460.6887813150388)*Mount_Control_B.RTin_k;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE_i[0], xnew,
                    sizeof(real_T)*2);
    }

    /* Update for DiscreteStateSpace: '<S4>/Discrete SS' */
    {
      real_T xnew[13];
      xnew[0] = (1.0)*Mount_Control_DW.DiscreteSS_DSTATE_l[0];
      xnew[0] += (-50416.062600024125)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow1[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow1[0];
        const real_T *pA1 = &Mount_Control_ConstP.DiscreteSS_A_k[1];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew1 = &xnew[1];
        int_T numNonZero = 11;
        *pxnew1 = (*pA1++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew1 += (*pA1++) * xd[*pAidx++];
        }
      }

      xnew[1] += (-3704.3814495289707)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow2[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow2[0];
        const real_T *pA13 = &Mount_Control_ConstP.DiscreteSS_A_k[13];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew2 = &xnew[2];
        int_T numNonZero = 11;
        *pxnew2 = (*pA13++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew2 += (*pA13++) * xd[*pAidx++];
        }
      }

      xnew[2] += (586.56505071926074)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow3[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow3[0];
        const real_T *pA25 = &Mount_Control_ConstP.DiscreteSS_A_k[25];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew3 = &xnew[3];
        int_T numNonZero = 11;
        *pxnew3 = (*pA25++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew3 += (*pA25++) * xd[*pAidx++];
        }
      }

      xnew[3] += (3253.4183618295)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow4[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow4[0];
        const real_T *pA37 = &Mount_Control_ConstP.DiscreteSS_A_k[37];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew4 = &xnew[4];
        int_T numNonZero = 11;
        *pxnew4 = (*pA37++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew4 += (*pA37++) * xd[*pAidx++];
        }
      }

      xnew[4] += (-2141.2242596893866)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow5[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow5[0];
        const real_T *pA49 = &Mount_Control_ConstP.DiscreteSS_A_k[49];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew5 = &xnew[5];
        int_T numNonZero = 11;
        *pxnew5 = (*pA49++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew5 += (*pA49++) * xd[*pAidx++];
        }
      }

      xnew[5] += (-410.07434352845769)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow6[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow6[0];
        const real_T *pA61 = &Mount_Control_ConstP.DiscreteSS_A_k[61];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew6 = &xnew[6];
        int_T numNonZero = 11;
        *pxnew6 = (*pA61++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew6 += (*pA61++) * xd[*pAidx++];
        }
      }

      xnew[6] += (1160.5679608156895)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow7[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow7[0];
        const real_T *pA73 = &Mount_Control_ConstP.DiscreteSS_A_k[73];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew7 = &xnew[7];
        int_T numNonZero = 11;
        *pxnew7 = (*pA73++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew7 += (*pA73++) * xd[*pAidx++];
        }
      }

      xnew[7] += (401.96388971624651)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow8[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow8[0];
        const real_T *pA85 = &Mount_Control_ConstP.DiscreteSS_A_k[85];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew8 = &xnew[8];
        int_T numNonZero = 11;
        *pxnew8 = (*pA85++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew8 += (*pA85++) * xd[*pAidx++];
        }
      }

      xnew[8] += (898.04096629569267)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow9[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
          12 };

        const int_T *pAidx = &colAidxRow9[0];
        const real_T *pA97 = &Mount_Control_ConstP.DiscreteSS_A_k[97];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew9 = &xnew[9];
        int_T numNonZero = 11;
        *pxnew9 = (*pA97++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew9 += (*pA97++) * xd[*pAidx++];
        }
      }

      xnew[9] += (-12.03202572711723)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow10[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
          11, 12 };

        const int_T *pAidx = &colAidxRow10[0];
        const real_T *pA109 = &Mount_Control_ConstP.DiscreteSS_A_k[109];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew10 = &xnew[10];
        int_T numNonZero = 11;
        *pxnew10 = (*pA109++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew10 += (*pA109++) * xd[*pAidx++];
        }
      }

      xnew[10] += (1552.4015588756)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow11[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
          11, 12 };

        const int_T *pAidx = &colAidxRow11[0];
        const real_T *pA121 = &Mount_Control_ConstP.DiscreteSS_A_k[121];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew11 = &xnew[11];
        int_T numNonZero = 11;
        *pxnew11 = (*pA121++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew11 += (*pA121++) * xd[*pAidx++];
        }
      }

      xnew[11] += (-78.728162089441525)*Mount_Control_B.RTin_g;

      {
        static const int_T colAidxRow12[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
          11, 12 };

        const int_T *pAidx = &colAidxRow12[0];
        const real_T *pA133 = &Mount_Control_ConstP.DiscreteSS_A_k[133];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_l[0];
        real_T *pxnew12 = &xnew[12];
        int_T numNonZero = 11;
        *pxnew12 = (*pA133++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew12 += (*pA133++) * xd[*pAidx++];
        }
      }

      xnew[12] += (-111.88680485620571)*Mount_Control_B.RTin_g;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE_l[0], xnew,
                    sizeof(real_T)*13);
    }

    /* Update for DiscreteStateSpace: '<S5>/Discrete SS' */
    {
      real_T xnew[2];
      xnew[0] = (0.89737819299898691)*Mount_Control_DW.DiscreteSS_DSTATE_b[0]
        + (0.041723174425305744)*Mount_Control_DW.DiscreteSS_DSTATE_b[1];
      xnew[0] += (-2899.709013113662)*Mount_Control_B.RTin_o;
      xnew[1] = (-0.041723174425305778)*Mount_Control_DW.DiscreteSS_DSTATE_b[0]
        + (0.98082454184959844)*Mount_Control_DW.DiscreteSS_DSTATE_b[1];
      xnew[1] += (-406.71497253110226)*Mount_Control_B.RTin_o;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE_b[0], xnew,
                    sizeof(real_T)*2);
    }

    /* Update for DiscreteStateSpace: '<S6>/Discrete SS' */
    {
      real_T xnew[8];
      xnew[0] = (1.0)*Mount_Control_DW.DiscreteSS_DSTATE_m[0];
      xnew[0] += (-664.69108805703058)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow1[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow1[0];
        const real_T *pA1 = &Mount_Control_ConstP.DiscreteSS_A_e[1];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew1 = &xnew[1];
        int_T numNonZero = 6;
        *pxnew1 = (*pA1++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew1 += (*pA1++) * xd[*pAidx++];
        }
      }

      xnew[1] += (-1951.3058211828352)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow2[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow2[0];
        const real_T *pA8 = &Mount_Control_ConstP.DiscreteSS_A_e[8];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew2 = &xnew[2];
        int_T numNonZero = 6;
        *pxnew2 = (*pA8++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew2 += (*pA8++) * xd[*pAidx++];
        }
      }

      xnew[2] += (1023.6997071541452)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow3[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow3[0];
        const real_T *pA15 = &Mount_Control_ConstP.DiscreteSS_A_e[15];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew3 = &xnew[3];
        int_T numNonZero = 6;
        *pxnew3 = (*pA15++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew3 += (*pA15++) * xd[*pAidx++];
        }
      }

      xnew[3] += (88.6156936051543)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow4[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow4[0];
        const real_T *pA22 = &Mount_Control_ConstP.DiscreteSS_A_e[22];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew4 = &xnew[4];
        int_T numNonZero = 6;
        *pxnew4 = (*pA22++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew4 += (*pA22++) * xd[*pAidx++];
        }
      }

      xnew[4] += (-84.669444418100255)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow5[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow5[0];
        const real_T *pA29 = &Mount_Control_ConstP.DiscreteSS_A_e[29];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew5 = &xnew[5];
        int_T numNonZero = 6;
        *pxnew5 = (*pA29++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew5 += (*pA29++) * xd[*pAidx++];
        }
      }

      xnew[5] += (-0.76460636181982622)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow6[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow6[0];
        const real_T *pA36 = &Mount_Control_ConstP.DiscreteSS_A_e[36];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew6 = &xnew[6];
        int_T numNonZero = 6;
        *pxnew6 = (*pA36++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew6 += (*pA36++) * xd[*pAidx++];
        }
      }

      xnew[6] += (28.817311108686653)*Mount_Control_B.RTin_b;

      {
        static const int_T colAidxRow7[7] = { 1, 2, 3, 4, 5, 6, 7 };

        const int_T *pAidx = &colAidxRow7[0];
        const real_T *pA43 = &Mount_Control_ConstP.DiscreteSS_A_e[43];
        const real_T *xd = &Mount_Control_DW.DiscreteSS_DSTATE_m[0];
        real_T *pxnew7 = &xnew[7];
        int_T numNonZero = 6;
        *pxnew7 = (*pA43++) * xd[*pAidx++];
        while (numNonZero--) {
          *pxnew7 += (*pA43++) * xd[*pAidx++];
        }
      }

      xnew[7] += (62.581444159302642)*Mount_Control_B.RTin_b;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE_m[0], xnew,
                    sizeof(real_T)*8);
    }

    /* Update for DiscreteStateSpace: '<S7>/Discrete SS' */
    {
      real_T xnew[2];
      xnew[0] = (0.89737819299898691)*Mount_Control_DW.DiscreteSS_DSTATE_h[0]
        + (0.041723174425305834)*Mount_Control_DW.DiscreteSS_DSTATE_h[1];
      xnew[0] += (-412.8900049313084)*Mount_Control_B.RTin_c;
      xnew[1] = (-0.041723174425305785)*Mount_Control_DW.DiscreteSS_DSTATE_h[0]
        + (0.98082454184959844)*Mount_Control_DW.DiscreteSS_DSTATE_h[1];
      xnew[1] += (-57.912206450565371)*Mount_Control_B.RTin_c;
      (void) memcpy(&Mount_Control_DW.DiscreteSS_DSTATE_h[0], xnew,
                    sizeof(real_T)*2);
    }
  }

  /* End of RateTransition: '<S7>/RTin' */
  rate_scheduler();
}

/* Model initialize function */
void Mount_Control_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Mount_Control_M, 0,
                sizeof(RT_MODEL_Mount_Control_T));

  /* block I/O */
  (void) memset(((void *) &Mount_Control_B), 0,
                sizeof(B_Mount_Control_T));

  /* states (dwork) */
  (void) memset((void *)&Mount_Control_DW, 0,
                sizeof(DW_Mount_Control_T));

  /* external inputs */
  (void)memset(&Mount_Control_U, 0, sizeof(ExtU_Mount_Control_T));

  /* external outputs */
  (void) memset(&Mount_Control_Y.Mount_cmd[0], 0,
                3U*sizeof(real_T));
}

/* Model terminate function */
void Mount_Control_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
