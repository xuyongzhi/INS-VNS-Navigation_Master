% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1780.450477158522500 ; 1780.068667379978700 ];

%-- Principal point:
cc = [ 690.352192640707130 ; 549.946304248088150 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.084280570703405 ; 0.148726145820102 ; -0.002897371075971 ; -0.002081937423982 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 5.539749811674626 ; 5.581864857355911 ];

%-- Principal point uncertainty:
cc_error = [ 7.861691857378122 ; 5.461506771840040 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013274940310358 ; 0.071661213829127 ; 0.000931923863828 ; 0.001369266996162 ; 0.000000000000000 ];

%-- Image size:
nx = 1392;
ny = 1040;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 47;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.112476e+000 ; -2.144335e+000 ; 1.258713e-001 ];
Tc_1  = [ -3.924098e+002 ; -2.112259e+002 ; 1.292948e+003 ];
omc_error_1 = [ 3.259142e-003 ; 3.026789e-003 ; 6.453535e-003 ];
Tc_error_1  = [ 5.718227e+000 ; 4.025988e+000 ; 4.457149e+000 ];

%-- Image #2:
omc_2 = [ NaN ; NaN ; NaN ];
Tc_2  = [ NaN ; NaN ; NaN ];
omc_error_2 = [ NaN ; NaN ; NaN ];
Tc_error_2  = [ NaN ; NaN ; NaN ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ -1.796073e+000 ; -1.989419e+000 ; -4.330551e-001 ];
Tc_4  = [ -3.791524e+002 ; -2.552036e+002 ; 1.216422e+003 ];
omc_error_4 = [ 2.273219e-003 ; 3.412120e-003 ; 5.404434e-003 ];
Tc_error_4  = [ 5.419549e+000 ; 3.835695e+000 ; 4.466852e+000 ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ -2.057412e+000 ; -2.129556e+000 ; -1.304864e-001 ];
Tc_6  = [ -3.782628e+002 ; -2.576922e+002 ; 1.547549e+003 ];
omc_error_6 = [ 3.528671e-003 ; 3.809465e-003 ; 7.557493e-003 ];
Tc_error_6  = [ 6.871231e+000 ; 4.828069e+000 ; 5.365593e+000 ];

%-- Image #7:
omc_7 = [ NaN ; NaN ; NaN ];
Tc_7  = [ NaN ; NaN ; NaN ];
omc_error_7 = [ NaN ; NaN ; NaN ];
Tc_error_7  = [ NaN ; NaN ; NaN ];

%-- Image #8:
omc_8 = [ NaN ; NaN ; NaN ];
Tc_8  = [ NaN ; NaN ; NaN ];
omc_error_8 = [ NaN ; NaN ; NaN ];
Tc_error_8  = [ NaN ; NaN ; NaN ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ 2.037008e+000 ; 2.127452e+000 ; 7.430270e-001 ];
Tc_10  = [ -3.222757e+002 ; -2.382989e+002 ; 1.351730e+003 ];
omc_error_10 = [ 3.991553e-003 ; 3.097288e-003 ; 6.175606e-003 ];
Tc_error_10  = [ 6.015736e+000 ; 4.209763e+000 ; 5.061286e+000 ];

%-- Image #11:
omc_11 = [ NaN ; NaN ; NaN ];
Tc_11  = [ NaN ; NaN ; NaN ];
omc_error_11 = [ NaN ; NaN ; NaN ];
Tc_error_11  = [ NaN ; NaN ; NaN ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ 1.885604e+000 ; 1.790435e+000 ; 6.944523e-002 ];
Tc_13  = [ -3.946344e+002 ; -2.443729e+002 ; 1.595197e+003 ];
omc_error_13 = [ 2.745638e-003 ; 3.504989e-003 ; 5.945300e-003 ];
Tc_error_13  = [ 7.062279e+000 ; 4.952827e+000 ; 5.644894e+000 ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ -1.955680e+000 ; -2.182581e+000 ; -7.747425e-001 ];
Tc_16  = [ -3.165029e+002 ; -2.309176e+002 ; 1.298017e+003 ];
omc_error_16 = [ 2.164323e-003 ; 3.622332e-003 ; 6.383634e-003 ];
Tc_error_16  = [ 5.782087e+000 ; 4.058166e+000 ; 4.837170e+000 ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ 1.883944e+000 ; 1.831047e+000 ; -1.442645e-001 ];
Tc_18  = [ -4.612781e+002 ; -2.529213e+002 ; 1.732104e+003 ];
omc_error_18 = [ 2.539119e-003 ; 3.700536e-003 ; 6.058802e-003 ];
Tc_error_18  = [ 7.672760e+000 ; 5.398010e+000 ; 6.001315e+000 ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

%-- Image #20:
omc_20 = [ NaN ; NaN ; NaN ];
Tc_20  = [ NaN ; NaN ; NaN ];
omc_error_20 = [ NaN ; NaN ; NaN ];
Tc_error_20  = [ NaN ; NaN ; NaN ];

%-- Image #21:
omc_21 = [ 1.928854e+000 ; 2.009224e+000 ; -5.146321e-001 ];
Tc_21  = [ -4.378766e+002 ; -2.455067e+002 ; 1.855053e+003 ];
omc_error_21 = [ 2.253122e-003 ; 3.798607e-003 ; 6.389106e-003 ];
Tc_error_21  = [ 8.216585e+000 ; 5.765881e+000 ; 5.969320e+000 ];

%-- Image #22:
omc_22 = [ NaN ; NaN ; NaN ];
Tc_22  = [ NaN ; NaN ; NaN ];
omc_error_22 = [ NaN ; NaN ; NaN ];
Tc_error_22  = [ NaN ; NaN ; NaN ];

%-- Image #23:
omc_23 = [ -1.982160e+000 ; -2.032655e+000 ; 4.693596e-001 ];
Tc_23  = [ -4.165579e+002 ; -2.875857e+002 ; 1.789702e+003 ];
omc_error_23 = [ 3.642909e-003 ; 2.828269e-003 ; 6.000183e-003 ];
Tc_error_23  = [ 7.927668e+000 ; 5.547991e+000 ; 5.748962e+000 ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ -1.847070e+000 ; -1.713109e+000 ; 3.614389e-001 ];
Tc_25  = [ -4.421800e+002 ; -2.644979e+002 ; 1.769665e+003 ];
omc_error_25 = [ 3.262446e-003 ; 2.989428e-003 ; 5.129620e-003 ];
Tc_error_25  = [ 7.832324e+000 ; 5.496245e+000 ; 5.699450e+000 ];

%-- Image #26:
omc_26 = [ NaN ; NaN ; NaN ];
Tc_26  = [ NaN ; NaN ; NaN ];
omc_error_26 = [ NaN ; NaN ; NaN ];
Tc_error_26  = [ NaN ; NaN ; NaN ];

%-- Image #27:
omc_27 = [ -1.824322e+000 ; -2.011836e+000 ; 9.018380e-001 ];
Tc_27  = [ -2.964835e+002 ; -2.769315e+002 ; 1.990913e+003 ];
omc_error_27 = [ 4.055608e-003 ; 2.716609e-003 ; 5.343514e-003 ];
Tc_error_27  = [ 8.808313e+000 ; 6.134862e+000 ; 5.903643e+000 ];

%-- Image #28:
omc_28 = [ -1.793738e+000 ; -2.173691e+000 ; 1.158950e+000 ];
Tc_28  = [ -2.260649e+002 ; -2.373872e+002 ; 2.073346e+003 ];
omc_error_28 = [ 4.546408e-003 ; 2.593522e-003 ; 5.428773e-003 ];
Tc_error_28  = [ 9.166006e+000 ; 6.380606e+000 ; 5.956433e+000 ];

%-- Image #29:
omc_29 = [ NaN ; NaN ; NaN ];
Tc_29  = [ NaN ; NaN ; NaN ];
omc_error_29 = [ NaN ; NaN ; NaN ];
Tc_error_29  = [ NaN ; NaN ; NaN ];

%-- Image #30:
omc_30 = [ NaN ; NaN ; NaN ];
Tc_30  = [ NaN ; NaN ; NaN ];
omc_error_30 = [ NaN ; NaN ; NaN ];
Tc_error_30  = [ NaN ; NaN ; NaN ];

%-- Image #31:
omc_31 = [ NaN ; NaN ; NaN ];
Tc_31  = [ NaN ; NaN ; NaN ];
omc_error_31 = [ NaN ; NaN ; NaN ];
Tc_error_31  = [ NaN ; NaN ; NaN ];

%-- Image #32:
omc_32 = [ -1.996901e+000 ; -2.180610e+000 ; 8.462259e-001 ];
Tc_32  = [ -3.101775e+002 ; -2.673224e+002 ; 2.030059e+003 ];
omc_error_32 = [ 4.260757e-003 ; 2.542228e-003 ; 6.098096e-003 ];
Tc_error_32  = [ 8.983451e+000 ; 6.258290e+000 ; 6.109197e+000 ];

%-- Image #33:
omc_33 = [ NaN ; NaN ; NaN ];
Tc_33  = [ NaN ; NaN ; NaN ];
omc_error_33 = [ NaN ; NaN ; NaN ];
Tc_error_33  = [ NaN ; NaN ; NaN ];

%-- Image #34:
omc_34 = [ NaN ; NaN ; NaN ];
Tc_34  = [ NaN ; NaN ; NaN ];
omc_error_34 = [ NaN ; NaN ; NaN ];
Tc_error_34  = [ NaN ; NaN ; NaN ];

%-- Image #35:
omc_35 = [ NaN ; NaN ; NaN ];
Tc_35  = [ NaN ; NaN ; NaN ];
omc_error_35 = [ NaN ; NaN ; NaN ];
Tc_error_35  = [ NaN ; NaN ; NaN ];

%-- Image #36:
omc_36 = [ NaN ; NaN ; NaN ];
Tc_36  = [ NaN ; NaN ; NaN ];
omc_error_36 = [ NaN ; NaN ; NaN ];
Tc_error_36  = [ NaN ; NaN ; NaN ];

%-- Image #37:
omc_37 = [ NaN ; NaN ; NaN ];
Tc_37  = [ NaN ; NaN ; NaN ];
omc_error_37 = [ NaN ; NaN ; NaN ];
Tc_error_37  = [ NaN ; NaN ; NaN ];

%-- Image #38:
omc_38 = [ NaN ; NaN ; NaN ];
Tc_38  = [ NaN ; NaN ; NaN ];
omc_error_38 = [ NaN ; NaN ; NaN ];
Tc_error_38  = [ NaN ; NaN ; NaN ];

%-- Image #39:
omc_39 = [ NaN ; NaN ; NaN ];
Tc_39  = [ NaN ; NaN ; NaN ];
omc_error_39 = [ NaN ; NaN ; NaN ];
Tc_error_39  = [ NaN ; NaN ; NaN ];

%-- Image #40:
omc_40 = [ -1.139661e+000 ; -2.558311e+000 ; -8.116899e-001 ];
Tc_40  = [ -2.294757e+002 ; -2.803770e+002 ; 1.323405e+003 ];
omc_error_40 = [ 1.643950e-003 ; 4.296492e-003 ; 5.404241e-003 ];
Tc_error_40  = [ 5.884805e+000 ; 4.137316e+000 ; 4.897204e+000 ];

%-- Image #41:
omc_41 = [ NaN ; NaN ; NaN ];
Tc_41  = [ NaN ; NaN ; NaN ];
omc_error_41 = [ NaN ; NaN ; NaN ];
Tc_error_41  = [ NaN ; NaN ; NaN ];

%-- Image #42:
omc_42 = [ -1.171573e+000 ; -2.826796e+000 ; 7.354851e-001 ];
Tc_42  = [ -1.796976e+002 ; -3.444889e+002 ; 1.967036e+003 ];
omc_error_42 = [ 3.328135e-003 ; 4.028336e-003 ; 6.040104e-003 ];
Tc_error_42  = [ 8.706986e+000 ; 6.063677e+000 ; 5.846434e+000 ];

%-- Image #43:
omc_43 = [ -2.199974e+000 ; -2.176369e+000 ; 1.086894e-004 ];
Tc_43  = [ -4.084468e+002 ; -2.224733e+002 ; 1.127764e+003 ];
omc_error_43 = [ 3.211304e-003 ; 2.853102e-003 ; 6.732925e-003 ];
Tc_error_43  = [ 5.017843e+000 ; 3.544482e+000 ; 4.112218e+000 ];

%-- Image #44:
omc_44 = [ NaN ; NaN ; NaN ];
Tc_44  = [ NaN ; NaN ; NaN ];
omc_error_44 = [ NaN ; NaN ; NaN ];
Tc_error_44  = [ NaN ; NaN ; NaN ];

%-- Image #45:
omc_45 = [ -2.103476e+000 ; -2.072497e+000 ; -1.347216e-001 ];
Tc_45  = [ -3.886065e+002 ; -2.032331e+002 ; 1.053497e+003 ];
omc_error_45 = [ 2.758928e-003 ; 2.961433e-003 ; 5.990218e-003 ];
Tc_error_45  = [ 4.691819e+000 ; 3.332188e+000 ; 3.848823e+000 ];

%-- Image #46:
omc_46 = [ NaN ; NaN ; NaN ];
Tc_46  = [ NaN ; NaN ; NaN ];
omc_error_46 = [ NaN ; NaN ; NaN ];
Tc_error_46  = [ NaN ; NaN ; NaN ];

%-- Image #47:
omc_47 = [ 1.928937e+000 ; 1.954670e+000 ; -5.196585e-001 ];
Tc_47  = [ -3.566019e+002 ; -2.368765e+002 ; 1.450903e+003 ];
omc_error_47 = [ 2.099370e-003 ; 3.501865e-003 ; 6.001760e-003 ];
Tc_error_47  = [ 6.437736e+000 ; 4.497061e+000 ; 4.628567e+000 ];

