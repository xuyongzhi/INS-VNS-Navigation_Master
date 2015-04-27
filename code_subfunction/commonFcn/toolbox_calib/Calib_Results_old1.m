% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1770.545062640278000 ; 1773.651532142868700 ];

%-- Principal point:
cc = [ 696.092246051838060 ; 535.728392663693400 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.095447468777489 ; 0.266004558338254 ; 0.000554022633332 ; -0.001027804668034 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.704153008677200 ; 2.532502925883887 ];

%-- Principal point uncertainty:
cc_error = [ 2.566095313544363 ; 2.082728407852853 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.004149972768456 ; 0.024610654319050 ; 0.000347474830771 ; 0.000438790528708 ; 0.000000000000000 ];

%-- Image size:
nx = 1392;
ny = 1040;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 17;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.982964e+000 ; -1.969318e+000 ; 4.601985e-001 ];
Tc_1  = [ -3.293367e+002 ; -6.790625e+002 ; 3.358088e+003 ];
omc_error_1 = [ 1.244396e-003 ; 9.610243e-004 ; 2.159002e-003 ];
Tc_error_1  = [ 4.906785e+000 ; 3.933558e+000 ; 4.694442e+000 ];

%-- Image #2:
omc_2 = [ -1.990977e+000 ; -1.976090e+000 ; 4.485925e-001 ];
Tc_2  = [ -4.164716e+002 ; -6.773064e+002 ; 3.792326e+003 ];
omc_error_2 = [ 1.312088e-003 ; 1.036756e-003 ; 2.262455e-003 ];
Tc_error_2  = [ 5.533155e+000 ; 4.451238e+000 ; 5.353144e+000 ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ -1.964748e+000 ; -2.036344e+000 ; 3.627632e-001 ];
Tc_4  = [ -6.392248e+002 ; -8.329491e+002 ; 3.314154e+003 ];
omc_error_4 = [ 1.301445e-003 ; 9.857718e-004 ; 2.199481e-003 ];
Tc_error_4  = [ 4.885699e+000 ; 3.920551e+000 ; 4.760902e+000 ];

%-- Image #5:
omc_5 = [ -1.953176e+000 ; -1.979308e+000 ; 3.882011e-001 ];
Tc_5  = [ -5.483964e+002 ; -8.840278e+002 ; 3.608732e+003 ];
omc_error_5 = [ 1.309765e-003 ; 9.918169e-004 ; 2.218494e-003 ];
Tc_error_5  = [ 5.313903e+000 ; 4.259407e+000 ; 5.119599e+000 ];

%-- Image #6:
omc_6 = [ NaN ; NaN ; NaN ];
Tc_6  = [ NaN ; NaN ; NaN ];
omc_error_6 = [ NaN ; NaN ; NaN ];
Tc_error_6  = [ NaN ; NaN ; NaN ];

%-- Image #7:
omc_7 = [ 2.129083e+000 ; 2.141775e+000 ; -7.956987e-002 ];
Tc_7  = [ -9.282184e+002 ; -2.641862e+002 ; 3.150848e+003 ];
omc_error_7 = [ 1.486872e-003 ; 1.749032e-003 ; 3.142447e-003 ];
Tc_error_7  = [ 4.580020e+000 ; 3.756234e+000 ; 4.801194e+000 ];

%-- Image #8:
omc_8 = [ 2.065596e+000 ; 2.344921e+000 ; 1.736318e-002 ];
Tc_8  = [ -6.034967e+002 ; -7.155516e+002 ; 2.959395e+003 ];
omc_error_8 = [ 1.330656e-003 ; 1.745060e-003 ; 3.304961e-003 ];
Tc_error_8  = [ 4.340825e+000 ; 3.498601e+000 ; 4.460739e+000 ];

%-- Image #9:
omc_9 = [ 2.363110e+000 ; 2.067382e+000 ; -1.661384e-002 ];
Tc_9  = [ -1.033674e+003 ; -5.314175e+002 ; 2.991166e+003 ];
omc_error_9 = [ 1.334557e-003 ; 1.502414e-003 ; 3.130258e-003 ];
Tc_error_9  = [ 4.376951e+000 ; 3.589392e+000 ; 4.595635e+000 ];

%-- Image #10:
omc_10 = [ 2.175229e+000 ; 2.185912e+000 ; -2.226385e-002 ];
Tc_10  = [ -8.352940e+002 ; -5.620448e+002 ; 2.463121e+003 ];
omc_error_10 = [ 1.083949e-003 ; 1.381275e-003 ; 2.582703e-003 ];
Tc_error_10  = [ 3.614612e+000 ; 2.943856e+000 ; 3.752170e+000 ];

%-- Image #11:
omc_11 = [ 2.012720e+000 ; 1.994842e+000 ; 3.241917e-001 ];
Tc_11  = [ -5.285378e+002 ; -5.707264e+002 ; 2.194314e+003 ];
omc_error_11 = [ 1.251723e-003 ; 1.226920e-003 ; 1.888978e-003 ];
Tc_error_11  = [ 3.228000e+000 ; 2.606263e+000 ; 3.312247e+000 ];

%-- Image #12:
omc_12 = [ 1.870341e+000 ; 1.906710e+000 ; 4.826935e-001 ];
Tc_12  = [ -4.279577e+002 ; -5.290010e+002 ; 2.236407e+003 ];
omc_error_12 = [ 1.294323e-003 ; 1.157993e-003 ; 1.696721e-003 ];
Tc_error_12  = [ 3.271738e+000 ; 2.641444e+000 ; 3.502294e+000 ];

%-- Image #13:
omc_13 = [ 2.173079e+000 ; 2.182353e+000 ; -7.071161e-002 ];
Tc_13  = [ -7.822967e+002 ; -5.084172e+002 ; 2.233759e+003 ];
omc_error_13 = [ 9.796344e-004 ; 1.254432e-003 ; 2.327519e-003 ];
Tc_error_13  = [ 3.274681e+000 ; 2.665988e+000 ; 3.411761e+000 ];

%-- Image #14:
omc_14 = [ -1.997566e+000 ; -1.970756e+000 ; 5.644675e-001 ];
Tc_14  = [ -8.627273e+002 ; -4.816686e+002 ; 2.625588e+003 ];
omc_error_14 = [ 1.353558e-003 ; 8.893170e-004 ; 1.878068e-003 ];
Tc_error_14  = [ 3.846858e+000 ; 3.149861e+000 ; 3.687766e+000 ];

%-- Image #15:
omc_15 = [ -1.943869e+000 ; -1.897080e+000 ; 6.824727e-001 ];
Tc_15  = [ -8.199752e+002 ; -4.058307e+002 ; 2.754378e+003 ];
omc_error_15 = [ 1.379022e-003 ; 9.101398e-004 ; 1.782322e-003 ];
Tc_error_15  = [ 4.020499e+000 ; 3.293039e+000 ; 3.729578e+000 ];

%-- Image #16:
omc_16 = [ -1.967602e+000 ; -1.888954e+000 ; 6.490758e-001 ];
Tc_16  = [ -9.224140e+002 ; -4.222895e+002 ; 2.732992e+003 ];
omc_error_16 = [ 1.393160e-003 ; 9.068526e-004 ; 1.802600e-003 ];
Tc_error_16  = [ 4.004287e+000 ; 3.287608e+000 ; 3.758560e+000 ];

%-- Image #17:
omc_17 = [ -2.004279e+000 ; -1.925921e+000 ; 6.070776e-001 ];
Tc_17  = [ -7.157689e+002 ; -4.050253e+002 ; 2.774439e+003 ];
omc_error_17 = [ 1.326204e-003 ; 8.853725e-004 ; 1.859617e-003 ];
Tc_error_17  = [ 4.034951e+000 ; 3.293960e+000 ; 3.804576e+000 ];

