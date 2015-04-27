% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1768.007295751431000 ; 1769.280804278628200 ];

%-- Principal point:
cc = [ 689.924026244641820 ; 543.770727958512230 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.093239938160486 ; 0.213152747894276 ; 0.001241942738812 ; -0.002513468555369 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.685176364514390 ; 4.789250084070493 ];

%-- Principal point uncertainty:
cc_error = [ 4.395241942047664 ; 4.151439866865519 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006785267896935 ; 0.028265887611756 ; 0.000997775998019 ; 0.000691671676519 ; 0.000000000000000 ];

%-- Image size:
nx = 1392;
ny = 1040;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 14;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.149534e+000 ; 2.167732e+000 ; -1.673261e-001 ];
Tc_1  = [ -1.943942e+002 ; -3.518933e+002 ; 1.518588e+003 ];
omc_error_1 = [ 1.626916e-003 ; 2.279276e-003 ; 4.331513e-003 ];
Tc_error_1  = [ 3.832353e+000 ; 3.524819e+000 ; 4.064986e+000 ];

%-- Image #2:
omc_2 = [ 1.909111e+000 ; 1.867603e+000 ; 3.626985e-001 ];
Tc_2  = [ -2.975290e+002 ; -3.535122e+002 ; 1.450882e+003 ];
omc_error_2 = [ 2.044557e-003 ; 2.122955e-003 ; 3.252700e-003 ];
Tc_error_2  = [ 3.660978e+000 ; 3.431515e+000 ; 3.907139e+000 ];

%-- Image #3:
omc_3 = [ 1.875096e+000 ; 1.825307e+000 ; 4.200777e-001 ];
Tc_3  = [ -3.186662e+002 ; -3.548903e+002 ; 1.328173e+003 ];
omc_error_3 = [ 2.033359e-003 ; 2.101722e-003 ; 3.129054e-003 ];
Tc_error_3  = [ 3.366728e+000 ; 3.155108e+000 ; 3.690355e+000 ];

%-- Image #4:
omc_4 = [ -1.934739e+000 ; -2.008331e+000 ; 6.658684e-001 ];
Tc_4  = [ -1.021672e+002 ; -3.514414e+002 ; 1.488985e+003 ];
omc_error_4 = [ 2.277727e-003 ; 1.302661e-003 ; 3.621628e-003 ];
Tc_error_4  = [ 3.762170e+000 ; 3.481857e+000 ; 3.679578e+000 ];

%-- Image #5:
omc_5 = [ -2.065668e+000 ; -2.121212e+000 ; 4.978435e-001 ];
Tc_5  = [ -7.976807e+001 ; -3.504075e+002 ; 1.569709e+003 ];
omc_error_5 = [ 2.237321e-003 ; 1.388079e-003 ; 3.890646e-003 ];
Tc_error_5  = [ 3.961383e+000 ; 3.647269e+000 ; 4.042026e+000 ];

%-- Image #6:
omc_6 = [ 1.924665e+000 ; 1.830288e+000 ; -3.713887e-002 ];
Tc_6  = [ -3.541258e+002 ; -3.473645e+002 ; 1.494244e+003 ];
omc_error_6 = [ 1.637007e-003 ; 2.189514e-003 ; 3.268376e-003 ];
Tc_error_6  = [ 3.764892e+000 ; 3.509062e+000 ; 4.164752e+000 ];

%-- Image #7:
omc_7 = [ 1.829917e+000 ; 1.657839e+000 ; 2.084106e-001 ];
Tc_7  = [ -3.621939e+002 ; -3.490450e+002 ; 1.349916e+003 ];
omc_error_7 = [ 1.872846e-003 ; 2.123945e-003 ; 2.950394e-003 ];
Tc_error_7  = [ 3.413566e+000 ; 3.194356e+000 ; 3.861294e+000 ];

%-- Image #8:
omc_8 = [ -2.032359e+000 ; -2.202669e+000 ; 8.393633e-001 ];
Tc_8  = [ -1.150348e+002 ; -3.439443e+002 ; 1.626172e+003 ];
omc_error_8 = [ 2.584997e-003 ; 1.179021e-003 ; 3.740475e-003 ];
Tc_error_8  = [ 4.100365e+000 ; 3.780839e+000 ; 4.105073e+000 ];

%-- Image #9:
omc_9 = [ -1.907519e+000 ; -2.122222e+000 ; 9.523085e-001 ];
Tc_9  = [ -7.745327e+001 ; -3.449340e+002 ; 1.509468e+003 ];
omc_error_9 = [ 2.588373e-003 ; 1.223372e-003 ; 3.561196e-003 ];
Tc_error_9  = [ 3.812792e+000 ; 3.516225e+000 ; 3.736360e+000 ];

%-- Image #10:
omc_10 = [ -1.761908e+000 ; -1.763385e+000 ; 7.419551e-001 ];
Tc_10  = [ -1.189554e+002 ; -3.490265e+002 ; 1.500258e+003 ];
omc_error_10 = [ 2.296873e-003 ; 1.426870e-003 ; 3.288460e-003 ];
Tc_error_10  = [ 3.787341e+000 ; 3.532719e+000 ; 3.525146e+000 ];

%-- Image #11:
omc_11 = [ 2.196831e+000 ; 2.208927e+000 ; 2.023168e-002 ];
Tc_11  = [ -2.567489e+002 ; -3.514285e+002 ; 1.356699e+003 ];
omc_error_11 = [ 1.895170e-003 ; 2.509761e-003 ; 4.915926e-003 ];
Tc_error_11  = [ 3.432632e+000 ; 3.170244e+000 ; 3.714575e+000 ];

%-- Image #12:
omc_12 = [ 1.836455e+000 ; 1.853112e+000 ; 6.171431e-001 ];
Tc_12  = [ -3.427610e+002 ; -3.529048e+002 ; 1.288555e+003 ];
omc_error_12 = [ 2.122665e-003 ; 2.039302e-003 ; 3.115896e-003 ];
Tc_error_12  = [ 3.286334e+000 ; 3.085983e+000 ; 3.676833e+000 ];

%-- Image #13:
omc_13 = [ 1.980844e+000 ; 1.995335e+000 ; 4.139954e-001 ];
Tc_13  = [ -3.960847e+002 ; -3.519790e+002 ; 1.405944e+003 ];
omc_error_13 = [ 2.071234e-003 ; 2.299313e-003 ; 3.561607e-003 ];
Tc_error_13  = [ 3.576041e+000 ; 3.364581e+000 ; 3.954056e+000 ];

%-- Image #14:
omc_14 = [ -1.969696e+000 ; -1.977372e+000 ; 4.742378e-001 ];
Tc_14  = [ -1.326234e+002 ; -3.486350e+002 ; 1.559651e+003 ];
omc_error_14 = [ 2.114202e-003 ; 1.377497e-003 ; 3.796304e-003 ];
Tc_error_14  = [ 3.931462e+000 ; 3.654383e+000 ; 3.865967e+000 ];

