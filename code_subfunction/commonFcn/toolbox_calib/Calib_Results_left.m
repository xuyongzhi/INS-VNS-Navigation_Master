% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1799.775768376820700 ; 1799.555870039128400 ];

%-- Principal point:
cc = [ 643.021108797809350 ; 527.490137421381860 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.085536904795936 ; 0.189669626363282 ; 0.000790070454483 ; -0.001469572265573 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.908862111929724 ; 4.608143005085156 ];

%-- Principal point uncertainty:
cc_error = [ 5.696422139685084 ; 4.820155579797166 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006528595196140 ; 0.024541621398898 ; 0.000889568975538 ; 0.000855990034818 ; 0.000000000000000 ];

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
omc_1 = [ 2.163378e+000 ; 2.161151e+000 ; -1.854714e-001 ];
Tc_1  = [ 3.029769e+001 ; -3.416574e+002 ; 1.527920e+003 ];
omc_error_1 = [ 2.068294e-003 ; 2.553548e-003 ; 5.015659e-003 ];
Tc_error_1  = [ 4.900464e+000 ; 4.046917e+000 ; 4.141066e+000 ];

%-- Image #2:
omc_2 = [ 1.919565e+000 ; 1.863564e+000 ; 3.444504e-001 ];
Tc_2  = [ -7.345122e+001 ; -3.425941e+002 ; 1.460949e+003 ];
omc_error_2 = [ 2.314249e-003 ; 2.339417e-003 ; 3.847315e-003 ];
Tc_error_2  = [ 4.700851e+000 ; 3.903738e+000 ; 3.876056e+000 ];

%-- Image #3:
omc_3 = [ 1.885231e+000 ; 1.821339e+000 ; 4.017048e-001 ];
Tc_3  = [ -9.597303e+001 ; -3.444102e+002 ; 1.337813e+003 ];
omc_error_3 = [ 2.325249e-003 ; 2.358069e-003 ; 3.731940e-003 ];
Tc_error_3  = [ 4.321798e+000 ; 3.575916e+000 ; 3.606239e+000 ];

%-- Image #4:
omc_4 = [ -1.938598e+000 ; -1.990030e+000 ; 6.813275e-001 ];
Tc_4  = [ 1.221235e+002 ; -3.418993e+002 ; 1.495495e+003 ];
omc_error_4 = [ 2.762592e-003 ; 1.807819e-003 ; 4.619109e-003 ];
Tc_error_4  = [ 4.801352e+000 ; 3.987925e+000 ; 3.640037e+000 ];

%-- Image #5:
omc_5 = [ -2.069407e+000 ; -2.103036e+000 ; 5.159230e-001 ];
Tc_5  = [ 1.454354e+002 ; -3.408056e+002 ; 1.577023e+003 ];
omc_error_5 = [ 2.789065e-003 ; 1.967254e-003 ; 4.850283e-003 ];
Tc_error_5  = [ 5.053166e+000 ; 4.180012e+000 ; 4.118992e+000 ];

%-- Image #6:
omc_6 = [ 1.933646e+000 ; 1.825854e+000 ; -5.597421e-002 ];
Tc_6  = [ -1.295748e+002 ; -3.356366e+002 ; 1.505443e+003 ];
omc_error_6 = [ 1.970508e-003 ; 2.481986e-003 ; 3.908003e-003 ];
Tc_error_6  = [ 4.831922e+000 ; 3.987721e+000 ; 4.179250e+000 ];

%-- Image #7:
omc_7 = [ 1.837596e+000 ; 1.654730e+000 ; 1.889686e-001 ];
Tc_7  = [ -1.391693e+002 ; -3.379955e+002 ; 1.360309e+003 ];
omc_error_7 = [ 2.200376e-003 ; 2.462430e-003 ; 3.557891e-003 ];
Tc_error_7  = [ 4.387124e+000 ; 3.614438e+000 ; 3.810610e+000 ];

%-- Image #8:
omc_8 = [ -2.034302e+000 ; -2.184145e+000 ; 8.560302e-001 ];
Tc_8  = [ 1.108037e+002 ; -3.336336e+002 ; 1.633852e+003 ];
omc_error_8 = [ 3.139882e-003 ; 1.549247e-003 ; 4.575880e-003 ];
Tc_error_8  = [ 5.231354e+000 ; 4.332389e+000 ; 4.111310e+000 ];

%-- Image #9:
omc_9 = [ -1.909341e+000 ; -2.103071e+000 ; 9.666568e-001 ];
Tc_9  = [ 1.470957e+002 ; -3.356170e+002 ; 1.515928e+003 ];
omc_error_9 = [ 3.151537e-003 ; 1.604122e-003 ; 4.407698e-003 ];
Tc_error_9  = [ 4.862037e+000 ; 4.035978e+000 ; 3.720806e+000 ];

%-- Image #10:
omc_10 = [ -1.748929e+000 ; -1.745400e+000 ; 7.703782e-001 ];
Tc_10  = [ -7.901160e+001 ; -3.353403e+002 ; 1.528845e+003 ];
omc_error_10 = [ 2.740421e-003 ; 2.012872e-003 ; 4.314910e-003 ];
Tc_error_10  = [ 4.921801e+000 ; 4.108589e+000 ; 3.499168e+000 ];

%-- Image #11:
omc_11 = [ 2.212552e+000 ; 2.202395e+000 ; 6.455611e-003 ];
Tc_11  = [ -3.380211e+001 ; -3.414005e+002 ; 1.365388e+003 ];
omc_error_11 = [ 2.380524e-003 ; 2.688145e-003 ; 5.432527e-003 ];
Tc_error_11  = [ 4.389869e+000 ; 3.632330e+000 ; 3.648636e+000 ];

%-- Image #12:
omc_12 = [ 1.847379e+000 ; 1.848712e+000 ; 5.987113e-001 ];
Tc_12  = [ -1.204574e+002 ; -3.422431e+002 ; 1.297645e+003 ];
omc_error_12 = [ 2.415257e-003 ; 2.297727e-003 ; 3.698114e-003 ];
Tc_error_12  = [ 4.207673e+000 ; 3.482626e+000 ; 3.486207e+000 ];

%-- Image #13:
omc_13 = [ 1.992950e+000 ; 1.989696e+000 ; 3.974246e-001 ];
Tc_13  = [ -1.724661e+002 ; -3.401942e+002 ; 1.416723e+003 ];
omc_error_13 = [ 2.296916e-003 ; 2.319587e-003 ; 3.929750e-003 ];
Tc_error_13  = [ 4.587256e+000 ; 3.804536e+000 ; 3.750998e+000 ];

%-- Image #14:
omc_14 = [ -1.975100e+000 ; -1.960113e+000 ; 4.903928e-001 ];
Tc_14  = [ 9.250883e+001 ; -3.384744e+002 ; 1.567217e+003 ];
omc_error_14 = [ 2.618393e-003 ; 2.006486e-003 ; 4.980884e-003 ];
Tc_error_14  = [ 5.024316e+000 ; 4.179711e+000 ; 3.830014e+000 ];

