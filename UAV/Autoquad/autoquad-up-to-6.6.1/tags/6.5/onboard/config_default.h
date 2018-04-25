/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#define DEFAULT_CONFIG_VERSION	    779

#define DEFAULT_RADIO_TYPE	    0		// 0 == Spektrum 11bit, 1 == Spektrum 10bit, 2 == Futaba, 3 == PWM

#define DEFAULT_CTRL_PID_TYPE	    0		// 0 == parallel TILT/RATE PIDs, 1 == cascading TILT/RATE PIDs

#define DEFAULT_CTRL_FACT_THRO	    0.70f	// user throttle multiplier
#define DEFAULT_CTRL_FACT_PITC	    0.05f	// user pitch multiplier
#define DEFAULT_CTRL_FACT_ROLL	    0.05f	// user roll multiplier
#define DEFAULT_CTRL_FACT_RUDD	    0.0004f	// user rudder multiplier
#define DEFAULT_CTRL_DEAD_BAND	    35.0f	// rc control dead band (for DVH & rudder control)
#define DEFAULT_CTRL_MIN_THROT	    20.0f	// minimum user throttle to activate motors
#define DEFAULT_CTRL_MAX	    300.0f	// maximum control applied to motors +- throttle
#define DEFAULT_CTRL_NAV_YAW_RT	    180.0f	// maximum navigation yaw rate deg/s

// TILT rate PID
#define DEFAULT_CTRL_TLT_RTE_P	    0.0f
#define DEFAULT_CTRL_TLT_RTE_I	    0.0f
#define DEFAULT_CTRL_TLT_RTE_D	    7180.0f
#define DEFAULT_CTRL_TLT_RTE_F	    0.25f
#define DEFAULT_CTRL_TLT_RTE_PM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_IM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_DM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_OM	    250.0f

// YAW rate PID
#define DEFAULT_CTRL_YAW_RTE_P	    300.0f
#define DEFAULT_CTRL_YAW_RTE_I	    0.15f
#define DEFAULT_CTRL_YAW_RTE_D	    50.0f
#define DEFAULT_CTRL_YAW_RTE_F	    0.25f
#define DEFAULT_CTRL_YAW_RTE_PM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_IM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_DM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_OM	    180.0f

// TILT angle PID
#define DEFAULT_CTRL_TLT_ANG_P	    60.0f
#define DEFAULT_CTRL_TLT_ANG_I	    0.0005f
#define DEFAULT_CTRL_TLT_ANG_D	    1744.0f
#define DEFAULT_CTRL_TLT_ANG_F	    0.25f
#define DEFAULT_CTRL_TLT_ANG_PM	    150.0f
#define DEFAULT_CTRL_TLT_ANG_IM	    75.0f
#define DEFAULT_CTRL_TLT_ANG_DM	    150.0f
#define DEFAULT_CTRL_TLT_ANG_OM	    250.0f

// YAW angle PID
#define DEFAULT_CTRL_YAW_ANG_P	    0.05f
#define DEFAULT_CTRL_YAW_ANG_I	    0.00002f
#define DEFAULT_CTRL_YAW_ANG_D	    0.0f
#define DEFAULT_CTRL_YAW_ANG_F	    0.0f
#define DEFAULT_CTRL_YAW_ANG_PM	    1.25f
#define DEFAULT_CTRL_YAW_ANG_IM	    0.04f
#define DEFAULT_CTRL_YAW_ANG_DM	    0.0f
#define DEFAULT_CTRL_YAW_ANG_OM	    1.25f

#define DEFAULT_GPS_BAUD_RATE	    230400
#define DEFAULT_GPS_RATE	    200		// ms per cycle (5Hz)

#define DEFAULT_MOT_FRAME	    1		// 0 == custom, 1 == '+' configuration, 2 == 'x', 3 == hex
#define DEFAULT_MOT_START	    1125
#define DEFAULT_MOT_MIN		    975
#define DEFAULT_MOT_MAX		    1950
#define DEFAULT_MOT_HOV_THROT	    500
#define DEFAULT_MOT_EXP_FACT	    0.0f
#define DEFAULT_MOT_EXP_MIN	    0.0f
#define DEFAULT_MOT_EXP_MAX	    0.0f

#define DEFAULT_MOT_PWRD_01_T	    0.0
#define DEFAULT_MOT_PWRD_01_P	    0.0
#define DEFAULT_MOT_PWRD_01_R	    0.0
#define DEFAULT_MOT_PWRD_01_Y	    0.0
#define DEFAULT_MOT_PWRD_02_T	    0.0
#define DEFAULT_MOT_PWRD_02_P	    0.0
#define DEFAULT_MOT_PWRD_02_R	    0.0
#define DEFAULT_MOT_PWRD_02_Y	    0.0
#define DEFAULT_MOT_PWRD_03_T	    0.0
#define DEFAULT_MOT_PWRD_03_P	    0.0
#define DEFAULT_MOT_PWRD_03_R	    0.0
#define DEFAULT_MOT_PWRD_03_Y	    0.0
#define DEFAULT_MOT_PWRD_04_T	    0.0
#define DEFAULT_MOT_PWRD_04_P	    0.0
#define DEFAULT_MOT_PWRD_04_R	    0.0
#define DEFAULT_MOT_PWRD_04_Y	    0.0
#define DEFAULT_MOT_PWRD_05_T	    0.0
#define DEFAULT_MOT_PWRD_05_P	    0.0
#define DEFAULT_MOT_PWRD_05_R	    0.0
#define DEFAULT_MOT_PWRD_05_Y	    0.0
#define DEFAULT_MOT_PWRD_06_T	    0.0
#define DEFAULT_MOT_PWRD_06_P	    0.0
#define DEFAULT_MOT_PWRD_06_R	    0.0
#define DEFAULT_MOT_PWRD_06_Y	    0.0
#define DEFAULT_MOT_PWRD_07_T	    0.0
#define DEFAULT_MOT_PWRD_07_P	    0.0
#define DEFAULT_MOT_PWRD_07_R	    0.0
#define DEFAULT_MOT_PWRD_07_Y	    0.0
#define DEFAULT_MOT_PWRD_08_T	    0.0
#define DEFAULT_MOT_PWRD_08_P	    0.0
#define DEFAULT_MOT_PWRD_08_R	    0.0
#define DEFAULT_MOT_PWRD_08_Y	    0.0
#define DEFAULT_MOT_PWRD_09_T	    0.0
#define DEFAULT_MOT_PWRD_09_P	    0.0
#define DEFAULT_MOT_PWRD_09_R	    0.0
#define DEFAULT_MOT_PWRD_09_Y	    0.0
#define DEFAULT_MOT_PWRD_10_T	    0.0
#define DEFAULT_MOT_PWRD_10_P	    0.0
#define DEFAULT_MOT_PWRD_10_R	    0.0
#define DEFAULT_MOT_PWRD_10_Y	    0.0
#define DEFAULT_MOT_PWRD_11_T	    0.0
#define DEFAULT_MOT_PWRD_11_P	    0.0
#define DEFAULT_MOT_PWRD_11_R	    0.0
#define DEFAULT_MOT_PWRD_11_Y	    0.0
#define DEFAULT_MOT_PWRD_12_T	    0.0
#define DEFAULT_MOT_PWRD_12_P	    0.0
#define DEFAULT_MOT_PWRD_12_R	    0.0
#define DEFAULT_MOT_PWRD_12_Y	    0.0
#define DEFAULT_MOT_PWRD_13_T	    0.0
#define DEFAULT_MOT_PWRD_13_P	    0.0
#define DEFAULT_MOT_PWRD_13_R	    0.0
#define DEFAULT_MOT_PWRD_13_Y	    0.0
#define DEFAULT_MOT_PWRD_14_T	    0.0
#define DEFAULT_MOT_PWRD_14_P	    0.0
#define DEFAULT_MOT_PWRD_14_R	    0.0
#define DEFAULT_MOT_PWRD_14_Y	    0.0


#define DEFAULT_DOWNLINK_BAUD	    115200

#define DEFAULT_TELEMETRY_RATE	    20		// loops between reports

#define DEFAULT_NAV_MAX_SPEED	    5.0f	// in m/s

// speed => tilt PID
#define DEFAULT_NAV_SPEED_P	    7.0f
#define DEFAULT_NAV_SPEED_I	    0.005f
#define DEFAULT_NAV_SPEED_PM	    20.0f
#define DEFAULT_NAV_SPEED_IM	    20.0f
#define DEFAULT_NAV_SPEED_OM	    30.0f

// distance => speed PID
#define DEFAULT_NAV_DIST_P	    0.5f
#define DEFAULT_NAV_DIST_I	    0.0f
#define DEFAULT_NAV_DIST_PM	    999.0f
#define DEFAULT_NAV_DIST_IM	    0.0f
#define DEFAULT_NAV_DIST_OM	    999.0f

// Altitude hold Speed PID
#define DEFAULT_NAV_ATL_SPED_P	    200.0f
#define DEFAULT_NAV_ATL_SPED_I	    1.2f
#define DEFAULT_NAV_ATL_SPED_PM	    150.0f
#define DEFAULT_NAV_ATL_SPED_IM	    600.0f
#define DEFAULT_NAV_ATL_SPED_OM	    600.0f

// Altitude hold Position PID
#define DEFAULT_NAV_ALT_POS_P	    0.25f
#define DEFAULT_NAV_ALT_POS_I	    0.0f
#define DEFAULT_NAV_ALT_POS_PM	    2.0f
#define DEFAULT_NAV_ALT_POS_IM	    0.0f
#define DEFAULT_NAV_ALT_POS_OM	    2.0f

#define DEFAULT_IMU_ROT		    +45.0		// degrees to rotate the IMU to align with the frame

#define DEFAULT_IMU_ACC_BIAS_X	    -1.646303293296
#define DEFAULT_IMU_ACC_BIAS_Y	    -1.647758547813
#define DEFAULT_IMU_ACC_BIAS_Z	    -1.656972469502
#define DEFAULT_IMU_ACC_BIAS1_X	    +1.698843630e-05
#define DEFAULT_IMU_ACC_BIAS1_Y	    -2.633256054e-05
#define DEFAULT_IMU_ACC_BIAS1_Z	    +5.305134683e-06
#define DEFAULT_IMU_ACC_BIAS2_X	    +1.099065085e-06
#define DEFAULT_IMU_ACC_BIAS2_Y	    -3.790755612e-07
#define DEFAULT_IMU_ACC_BIAS2_Z	    +1.493964013e-06
#define DEFAULT_IMU_ACC_BIAS3_X	    +5.049451315e-08
#define DEFAULT_IMU_ACC_BIAS3_Y	    -9.557781444e-08
#define DEFAULT_IMU_ACC_BIAS3_Z	    +1.554955106e-07

#define DEFAULT_IMU_ACC_SCAL_X	    +0.019435181772
#define DEFAULT_IMU_ACC_SCAL_Y	    +0.020107534159
#define DEFAULT_IMU_ACC_SCAL_Z	    +0.018908076812
#define DEFAULT_IMU_ACC_SCAL1_X	    +1.092031197e-05
#define DEFAULT_IMU_ACC_SCAL1_Y	    -7.817609441e-05
#define DEFAULT_IMU_ACC_SCAL1_Z	    -1.492628119e-06
#define DEFAULT_IMU_ACC_SCAL2_X	    -6.858854370e-07
#define DEFAULT_IMU_ACC_SCAL2_Y	    +6.351551904e-06
#define DEFAULT_IMU_ACC_SCAL2_Z	    -2.748601448e-07
#define DEFAULT_IMU_ACC_SCAL3_X	    +9.427540068e-10
#define DEFAULT_IMU_ACC_SCAL3_Y	    -1.714591736e-07
#define DEFAULT_IMU_ACC_SCAL3_Z	    +1.896782059e-08

#define DEFAULT_IMU_ACC_ALGN_XY	    +0.008845901554
#define DEFAULT_IMU_ACC_ALGN_XZ	    +0.004617727074
#define DEFAULT_IMU_ACC_ALGN_YX	    +0.009823775456
#define DEFAULT_IMU_ACC_ALGN_YZ	    +0.000274681390
#define DEFAULT_IMU_ACC_ALGN_ZX	    +0.002191875675
#define DEFAULT_IMU_ACC_ALGN_ZY	    -0.001480375290

#define DEFAULT_IMU_MAG_BIAS_X	    +0.002245023338
#define DEFAULT_IMU_MAG_BIAS_Y	    +0.048153341570
#define DEFAULT_IMU_MAG_BIAS_Z	    +0.031058781669
#define DEFAULT_IMU_MAG_BIAS1_X	    +1.894171071e-04
#define DEFAULT_IMU_MAG_BIAS1_Y	    +1.685155493e-04
#define DEFAULT_IMU_MAG_BIAS1_Z	    -1.067656912e-04
#define DEFAULT_IMU_MAG_BIAS2_X	    -1.872557347e-07
#define DEFAULT_IMU_MAG_BIAS2_Y	    -1.058228873e-05
#define DEFAULT_IMU_MAG_BIAS2_Z	    +9.710330598e-07
#define DEFAULT_IMU_MAG_BIAS3_X	    -2.948742777e-07
#define DEFAULT_IMU_MAG_BIAS3_Y	    -7.939735498e-07
#define DEFAULT_IMU_MAG_BIAS3_Z	    +6.450152297e-07

#define DEFAULT_IMU_MAG_SCAL_X	    +0.203449586600
#define DEFAULT_IMU_MAG_SCAL_Y	    +0.213204714724
#define DEFAULT_IMU_MAG_SCAL_Z	    +0.179515084604
#define DEFAULT_IMU_MAG_SCAL1_X	    -1.659341327e-04
#define DEFAULT_IMU_MAG_SCAL1_Y	    -2.399340432e-04
#define DEFAULT_IMU_MAG_SCAL1_Z	    -4.737563513e-04
#define DEFAULT_IMU_MAG_SCAL2_X	    -4.231269400e-06
#define DEFAULT_IMU_MAG_SCAL2_Y	    -4.797719958e-06
#define DEFAULT_IMU_MAG_SCAL2_Z	    +6.312346711e-08
#define DEFAULT_IMU_MAG_SCAL3_X	    -8.347036509e-07
#define DEFAULT_IMU_MAG_SCAL3_Y	    -9.639550738e-07
#define DEFAULT_IMU_MAG_SCAL3_Z	    -2.557594418e-07

#define DEFAULT_IMU_MAG_ALGN_XY	    -0.012551753359
#define DEFAULT_IMU_MAG_ALGN_XZ	    -0.025656958072
#define DEFAULT_IMU_MAG_ALGN_YX	    -0.033484201147
#define DEFAULT_IMU_MAG_ALGN_YZ	    +0.039333117868
#define DEFAULT_IMU_MAG_ALGN_ZX	    -0.069339097217
#define DEFAULT_IMU_MAG_ALGN_ZY	    -0.075123829376

#define DEFAULT_IMU_GYO_BIAS_X	    -1.332798435e+00
#define DEFAULT_IMU_GYO_BIAS_Y	    -1.351016012e+00
#define DEFAULT_IMU_GYO_BIAS_Z	    -1.327206609e+00
#define DEFAULT_IMU_GYO_BIAS1_X	    +1.811929025e-04
#define DEFAULT_IMU_GYO_BIAS1_Y	    -1.008877982e-04
#define DEFAULT_IMU_GYO_BIAS1_Z	    +4.508935312e-05
#define DEFAULT_IMU_GYO_BIAS2_X	    +9.375212006e-06
#define DEFAULT_IMU_GYO_BIAS2_Y	    -2.593755062e-06
#define DEFAULT_IMU_GYO_BIAS2_Z	    -5.053406575e-09
#define DEFAULT_IMU_GYO_BIAS3_X	    +2.268343556e-07
#define DEFAULT_IMU_GYO_BIAS3_Y	    -1.229831245e-07
#define DEFAULT_IMU_GYO_BIAS3_Z	    -2.281832596e-08

#define DEFAULT_IMU_GYO_SCAL_X	    +0.117429998776
#define DEFAULT_IMU_GYO_SCAL_Y	    +0.113568193249
#define DEFAULT_IMU_GYO_SCAL_Z	    +0.114568363368

#define DEFAULT_IMU_GYO_ALGN_XY	    -0.001412686163
#define DEFAULT_IMU_GYO_ALGN_XZ	    +0.000202120006
#define DEFAULT_IMU_GYO_ALGN_YX	    +0.001293166445
#define DEFAULT_IMU_GYO_ALGN_YZ	    -0.011873154183
#define DEFAULT_IMU_GYO_ALGN_ZX	    -0.006451915196
#define DEFAULT_IMU_GYO_ALGN_ZY	    -0.040268338334

#define DEFAULT_IMU_MAG_INCL	    -66.358701039003
#define DEFAULT_IMU_MAG_DECL	    -10.11f

#define DEFAULT_GMBL_PWM_MAX	    2250
#define DEFAULT_GMBL_PWM_MIN	    750
#define DEFAULT_GMBL_NTRL_PITCH	    1575
#define DEFAULT_GMBL_NTRL_ROLL	    1442
#define DEFAULT_GMBL_SCAL_PITCH	    (1.0f / 91.0f)
#define DEFAULT_GMBL_SCAL_ROLL	    (1.0f / 70.0f)
#define DEFAULT_GMBL_SLEW_RATE	    0.005f

#define DEFAULT_SPVR_LOW_BAT1	    3.5f	// cell volts
#define DEFAULT_SPVR_LOW_BAT2	    3.3f	// cell volts
/*
#define DEFAULT_UKF_VEL_Q               +1.7850e-05     // +0.000017849835       0.000014998572 +0.000000000082
#define DEFAULT_UKF_VEL_ALT_Q           +1.6361e-05     // +0.000016360889       0.000014984796 -0.000000001961
#define DEFAULT_UKF_POS_Q               +1.1986e+02     // +119.858284978688     0.000014991305 +0.01149712109621
#define DEFAULT_UKF_POS_ALT_Q           +3.5726e+01     // +35.725732132291      0.000014997820 +0.0012311066319
#define DEFAULT_UKF_ACC_BIAS_Q          +8.8128e-08     // +0.000000088128       0.000014917445 +0.000000000006
#define DEFAULT_UKF_GYO_BIAS_Q          +8.1026e-06     // +0.000008102591       0.000014999874 +0.000000000020
#define DEFAULT_UKF_QUAT_Q              +6.3381e-08     // +0.000000063381       0.000014996698 +0.000000000006
#define DEFAULT_UKF_PRES_ALT_Q          +9.5054e-01     // +0.950539491760       0.000015000001 -0.000000002210
#define DEFAULT_UKF_ACC_BIAS_V          +4.6635e-09     // +0.000000004663       0.000014982073 +0.000000000000
#define DEFAULT_UKF_GYO_BIAS_V          +5.4348e-11     // +0.000000000054       0.000014987804 +0.000000000000
#define DEFAULT_UKF_RATE_V              +8.3123e-09     // +0.000000008312       0.000014216278 +0.000000000001
#define DEFAULT_UKF_PRES_ALT_V          +6.2198e-09     // +0.000000006220       0.000014639232 +0.000000000001
#define DEFAULT_UKF_POS_V               +1.3213e-11     // +0.000000000013       0.000014965672 -0.000000000000
#define DEFAULT_UKF_VEL_V               +4.8886e-12     // +0.000000000005       0.000014995938 +0.000000000000
#define DEFAULT_UKF_ALT_POS_V           +1.6382e-12     // +0.000000000002       0.000014999883 -0.000000000000
#define DEFAULT_UKF_ALT_VEL_V           +3.6327e-11     // +0.000000000036       0.000014970403 -0.000000000000
#define DEFAULT_UKF_GPS_POS_N           +6.9339e-10     // +0.000000000693       0.000014997840 -0.000000000000
#define DEFAULT_UKF_GPS_POS_M_N         +1.4640e-08     // +0.000000014640       0.000014172153 +0.000000000001
#define DEFAULT_UKF_GPS_ALT_N           +8.3513e-09     // +0.000000008351       0.000014998315 -0.000000000001
#define DEFAULT_UKF_GPS_ALT_M_N         +2.3975e-08     // +0.000000023975       0.000014903865 -0.000000000000
#define DEFAULT_UKF_GPS_VEL_N           +3.1099e-06     // +0.000003109892       0.000013639938 +0.000000000044
#define DEFAULT_UKF_GPS_VEL_M_N         +8.8824e-07     // +0.000000888242       0.000014945468 +0.000000000007
#define DEFAULT_UKF_GPS_VD_N            +2.1276e-04     // +0.000212759861       0.000014999392 -0.000000000818
#define DEFAULT_UKF_GPS_VD_M_N          +1.3701e-06     // +0.000001370124       0.000015000000 +0.000000000000
#define DEFAULT_UKF_ALT_N               +2.9679e-05     // +0.000029678964       0.000014909641 -0.000000016376
#define DEFAULT_UKF_ACC_N               +1.9387e-08     // +0.000000019387       0.000014680501 -0.000000000002
#define DEFAULT_UKF_DIST_N              +1.9479e-06     // +0.000001947855       0.000012818091 +0.000000000257
#define DEFAULT_UKF_MAG_N               +4.2068e-05     // +0.000042068049       0.000014960041 -0.000000005364
#define DEFAULT_UKF_POS_DELAY           +7.1107e+03     // +7110.663710609745    0.000014999542 +0.055542009804749
#define DEFAULT_UKF_VEL_DELAY           -1.2509e+05     // -125094.292583522983  0.000004538149 +10.6874301460783000
*/

#define DEFAULT_UKF_VEL_Q		+6.9861e-06	// +0.000006986094	 0.000008284630 -0.000000178158
#define DEFAULT_UKF_VEL_ALT_Q		+1.4079e-05	// +0.000014079267	 0.000009791544 -0.000000088476
#define DEFAULT_UKF_POS_Q		+7.6956e+01	// +76.956013758078	 0.000005515702 +3.5221585142992
#define DEFAULT_UKF_POS_ALT_Q		+6.5830e+01	// +65.829689379671	 0.000009513413 -1.4045588290347
#define DEFAULT_UKF_ACC_BIAS_Q		+1.2705e-07	// +0.000000127053	 0.000009825083 +0.000000000613
#define DEFAULT_UKF_GYO_BIAS_Q		+5.0717e-06	// +0.000005071742	 0.000008777417 +0.000000060373
#define DEFAULT_UKF_QUAT_Q		+7.6126e-08	// +0.000000076126	 0.000008268599 +0.000000000006
#define DEFAULT_UKF_PRES_ALT_Q		+8.1440e-01	// +0.814400664958	 0.000009387677 -0.012213459714
#define DEFAULT_UKF_ACC_BIAS_V		+2.3980e-11	// +0.000000000024	 0.000010723092 -0.000000000000
#define DEFAULT_UKF_GYO_BIAS_V		+2.0645e-11	// +0.000000000021	 0.000010170254 -0.000000000001
#define DEFAULT_UKF_RATE_V		+4.1262e-09	// +0.000000004126	 0.000009352056 +0.000000000015
#define DEFAULT_UKF_PRES_ALT_V		+1.0294e-08	// +0.000000010294	 0.000009467923 -0.000000000078
#define DEFAULT_UKF_POS_V		+8.4432e-11	// +0.000000000084	 0.000009020525 +0.000000000000
#define DEFAULT_UKF_VEL_V		+1.6034e-11	// +0.000000000016	 0.000009871527 +0.000000000000
#define DEFAULT_UKF_ALT_POS_V		+6.1930e-13	// +0.000000000001	 0.000009246523 +0.000000000000
#define DEFAULT_UKF_ALT_VEL_V		+2.3861e-11	// +0.000000000024	 0.000009536499 +0.000000000000
#define DEFAULT_UKF_GPS_POS_N		+1.9786e-09	// +0.000000001979	 0.000007676041 -0.000000000017
#define DEFAULT_UKF_GPS_POS_M_N		+2.5346e-09	// +0.000000002535	 0.000009306523 +0.000000000080
#define DEFAULT_UKF_GPS_ALT_N		+1.9611e-08	// +0.000000019611	 0.000009352814 +0.000000000019
#define DEFAULT_UKF_GPS_ALT_M_N		+4.3243e-09	// +0.000000004324	 0.000007293794 -0.000000000028
#define DEFAULT_UKF_GPS_VEL_N		+5.9760e-06	// +0.000005976030	 0.000007947240 +0.000000041466
#define DEFAULT_UKF_GPS_VEL_M_N		+1.1956e-06	// +0.000001195557	 0.000009054273 -0.000000022516
#define DEFAULT_UKF_GPS_VD_N		+2.9347e-04	// +0.000293474575	 0.000009384798 +0.000001451206
#define DEFAULT_UKF_GPS_VD_M_N		+1.4114e-06	// +0.000001411353	 0.000009660760 +0.000000006032
#define DEFAULT_UKF_ALT_N		+2.2497e-05	// +0.000022496581	 0.000009060313 -0.000000068717
#define DEFAULT_UKF_ACC_N		+5.7378e-09	// +0.000000005738	 0.000008727542 +0.000000000186
#define DEFAULT_UKF_DIST_N		+4.0505e-06	// +0.000004050474	 0.000009163209 -0.000000018100
#define DEFAULT_UKF_MAG_N		+3.9315e-05	// +0.000039315467	 0.000008921520 -0.000000161858
#define DEFAULT_UKF_POS_DELAY		+2.0121e+03	// +2012.129621782368	 0.000099227182 +47.67786158114690
#define DEFAULT_UKF_VEL_DELAY		-1.0808e+05	// -108083.965722650159	 0.000009969907 +154.195266188661200

#define DEFAULT_VN100_MAG_BIAS_X	0.0f
#define DEFAULT_VN100_MAG_BIAS_Y	0.0f
#define DEFAULT_VN100_MAG_BIAS_Z	0.0f
#define DEFAULT_VN100_MAG_SCAL_X	1.0f
#define DEFAULT_VN100_MAG_SCAL_Y	1.0f
#define DEFAULT_VN100_MAG_SCAL_Z	1.0f
#define DEFAULT_VN100_MAG_ALGN_XY	0.0f
#define DEFAULT_VN100_MAG_ALGN_XZ	0.0f
#define DEFAULT_VN100_MAG_ALGN_YX	0.0f
#define DEFAULT_VN100_MAG_ALGN_YZ	0.0f
#define DEFAULT_VN100_MAG_ALGN_ZX	0.0f
#define DEFAULT_VN100_MAG_ALGN_ZY	0.0f
