import numpy as np
import os

cfd_cases = [
    "b2019_0z_0az_os_2ms",
    "b2019_0z_0az_os_7ms",
    "b2019_0z_0az_cd_12ms",
    "b2019_0z_0az_cd_17ms",
    "b2019_0z_45az_os_2ms",
    "b2019_0z_45az_os_7ms",
    "b2019_0z_45az_cd_12ms",
    "b2019_0z_45az_cd_17ms",
    "b2019_0z_90az_os_2ms",
    "b2019_0z_90az_os_7ms",
    "b2019_0z_90az_cd_12ms",
    "b2019_0z_90az_cd_17ms",
    "b2019_0z_135az_os_2ms",
    "b2019_0z_135az_os_7ms",
    "b2019_0z_135az_cd_12ms",
    "b2019_0z_135az_cd_17ms",
    "b2019_0z_180az_os_2ms",
    "b2019_0z_180az_os_7ms",
    "b2019_0z_180az_cd_12ms",
    "b2019_0z_180az_cd_17ms",
    "b2019_30z_0az_os_2ms",
    "b2019_30z_0az_os_7ms",
    "b2019_30z_0az_cd_12ms",
    "b2019_30z_0az_cd_17ms",
    "b2019_30z_45az_os_2ms",
    "b2019_30z_45az_os_7ms",
    "b2019_30z_45az_cd_12ms",
    "b2019_30z_45az_cd_17ms",
    "b2019_30z_90az_os_2ms",
    "b2019_30z_90az_os_7ms",
    "b2019_30z_90az_cd_12ms",
    "b2019_30z_90az_cd_17ms",
    "b2019_30z_135az_os_2ms",
    "b2019_30z_135az_os_7ms",
    "b2019_30z_135az_cd_12ms",
    "b2019_30z_135az_cd_17ms",
    "b2019_30z_180az_os_2ms",
    "b2019_30z_180az_os_7ms",
    "b2019_30z_180az_cd_12ms",
    "b2019_30z_180az_cd_17ms",
    "b2019_60z_0az_os_2ms",
    "b2019_60z_0az_os_7ms",
    "b2019_60z_0az_cd_12ms",
    "b2019_60z_0az_cd_17ms",
    "b2019_60z_45az_os_2ms",
    "b2019_60z_45az_os_7ms",
    "b2019_60z_45az_cd_12ms",
    "b2019_60z_45az_cd_17ms",
    "b2019_60z_90az_os_2ms",
    "b2019_60z_90az_os_7ms",
    "b2019_60z_90az_cd_12ms",
    "b2019_60z_90az_cd_17ms",
    "b2019_60z_135az_os_2ms",
    "b2019_60z_135az_os_7ms",
    "b2019_60z_135az_cd_12ms",
    "b2019_60z_135az_cd_17ms",
    "b2019_60z_180az_os_2ms",
    "b2019_60z_180az_os_7ms",
    "b2019_60z_180az_cd_12ms",
    "b2019_60z_180az_cd_17ms",
];


dataroot = "/fsx/Baseline2020"
r2a = 180*3600e3/np.pi
D_jitter = np.load('linear_jitter.npz')
for cfd_case in cfd_cases:
    filename = "MT_FSM_Jitter_FSM_MountCtrl.pkl"
    datapath = os.path.join(dataroot,cfd_case,filename)
    data = np.load(datapath,allow_pickle=True)
    py_jitter = r2a*data['Data']['Pupil'][:,2000*30:].std(1)

    filename = "wind_loading.data.pkl"
    datapath = os.path.join(dataroot,cfd_case,filename)
    data = dict(np.load(datapath,allow_pickle=True))
    m1_rbm = np.vstack([x[1] for x in data['OSSM1Lcl']['data']])
    m2_rbm = np.vstack([x[1] for x in data['MCM2Lcl6D']['data']])
    rbm = np.hstack([m1_rbm,m2_rbm])
    rs_jitter = r2a*(D_jitter['D_tt']@rbm.T)[:,2000*30:].std(1)

    err = np.hypot(*(rs_jitter-py_jitter))
    print("{:24} [{:4.0f},{:4.0f}]  [{:4.0f},{:4.0f}] ({:4.3f})".format(cfd_case,py_jitter[0],py_jitter[1],rs_jitter[0],rs_jitter[1],err))


"""
b2019_0z_0az_os_2ms      [   2,   1]  [   2,   1] (0.056)
b2019_0z_0az_os_7ms      [  25,  11]  [  25,  11] (0.300)
b2019_0z_0az_cd_12ms     [   4,   2]  [   4,   2] (0.153)
b2019_0z_0az_cd_17ms     [   9,   4]  [   9,   4] (0.456)
b2019_0z_45az_os_2ms     [   1,   0]  [   1,   0] (0.036)
b2019_0z_45az_os_7ms     [   7,   7]  [   7,   6] (0.187)
b2019_0z_45az_cd_12ms    [   4,   2]  [   4,   2] (0.332)
b2019_0z_45az_cd_17ms    [   9,   6]  [   9,   6] (0.210)
b2019_0z_90az_os_2ms     [   1,   1]  [   1,   1] (0.019)
b2019_0z_90az_os_7ms     [  12,   9]  [  12,   9] (0.206)
b2019_0z_90az_cd_12ms    [   5,   7]  [   5,   7] (0.656)
b2019_0z_90az_cd_17ms    [  14,  17]  [  14,  16] (0.715)
b2019_0z_135az_os_2ms    [   1,   1]  [   1,   1] (0.087)
b2019_0z_135az_os_7ms    [  13,  10]  [  13,  10] (0.221)
b2019_0z_135az_cd_12ms   [  11,   8]  [  11,   8] (0.229)
b2019_0z_135az_cd_17ms   [  27,  19]  [  27,  18] (0.418)
b2019_0z_180az_os_2ms    [   1,   1]  [   1,   1] (0.029)
b2019_0z_180az_os_7ms    [  16,   9]  [  16,   9] (0.244)
b2019_0z_180az_cd_12ms   [  19,  12]  [  18,  11] (0.598)
b2019_0z_180az_cd_17ms   [  43,  25]  [  42,  24] (1.538)
b2019_30z_0az_os_2ms     [   3,   1]  [   3,   1] (0.064)
b2019_30z_0az_os_7ms     [  20,  10]  [  20,  10] (0.111)
b2019_30z_0az_cd_12ms    [   6,   4]  [   6,   4] (0.068)
b2019_30z_0az_cd_17ms    [  14,   8]  [  13,   7] (0.407)
b2019_30z_45az_os_2ms    [   0,   1]  [   0,   1] (0.018)
b2019_30z_45az_os_7ms    [  10,   8]  [   9,   8] (0.104)
b2019_30z_45az_cd_12ms   [   4,   3]  [   3,   3] (0.577)
b2019_30z_45az_cd_17ms   [   8,   7]  [   7,   7] (0.539)
b2019_30z_90az_os_2ms    [   1,   1]  [   1,   1] (0.020)
b2019_30z_90az_os_7ms    [  14,  12]  [  14,  12] (0.335)
b2019_30z_90az_cd_12ms   [   6,   6]  [   6,   6] (0.254)
b2019_30z_90az_cd_17ms   [  16,  15]  [  15,  15] (0.814)
b2019_30z_135az_os_2ms   [   1,   1]  [   1,   1] (0.024)
b2019_30z_135az_os_7ms   [  12,   9]  [  12,   9] (0.609)
b2019_30z_135az_cd_12ms  [  14,  13]  [  14,  13] (0.280)
b2019_30z_135az_cd_17ms  [  29,  27]  [  29,  27] (0.460)
b2019_30z_180az_os_2ms   [   1,   0]  [   0,   0] (0.018)
b2019_30z_180az_os_7ms   [  11,   7]  [  10,   6] (0.360)
b2019_30z_180az_cd_12ms  [  17,   9]  [  16,   9] (0.642)
b2019_30z_180az_cd_17ms  [  36,  18]  [  35,  18] (1.026)
b2019_60z_0az_os_2ms     [   2,   1]  [   2,   1] (0.072)
b2019_60z_0az_os_7ms     [  23,  11]  [  23,  11] (0.135)
b2019_60z_0az_cd_12ms    [  70,  31]  [  70,  30] (0.464)
b2019_60z_0az_cd_17ms    [ 170,  62]  [ 170,  62] (0.520)
b2019_60z_45az_os_2ms    [   0,   1]  [   0,   1] (0.019)
b2019_60z_45az_os_7ms    [   8,   9]  [   8,   9] (0.086)
b2019_60z_45az_cd_12ms   [  13,  11]  [  12,  11] (0.408)
b2019_60z_45az_cd_17ms   [  33,  23]  [  33,  22] (0.900)
b2019_60z_90az_os_2ms    [   1,   1]  [   1,   1] (0.008)
b2019_60z_90az_os_7ms    [  10,  12]  [  10,  12] (0.176)
b2019_60z_90az_cd_12ms   [   7,   7]  [   7,   7] (0.372)
b2019_60z_90az_cd_17ms   [  20,  17]  [  19,  17] (1.146)
b2019_60z_135az_os_2ms   [   1,   1]  [   1,   1] (0.011)
b2019_60z_135az_os_7ms   [  11,   7]  [  10,   7] (0.529)
b2019_60z_135az_cd_12ms  [  26,  15]  [  26,  15] (0.747)
b2019_60z_135az_cd_17ms  [  62,  33]  [  61,  32] (1.353)
b2019_60z_180az_os_2ms   [   0,   0]  [   0,   0] (0.031)
b2019_60z_180az_os_7ms   [   9,   6]  [   9,   6] (0.284)
b2019_60z_180az_cd_12ms  [  25,  16]  [  25,  16] (0.410)
b2019_60z_180az_cd_17ms  [  61,  37]  [  60,  36] (1.081)
"""
