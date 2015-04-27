% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1790.173735574028600 ; 1789.459735133257000 ];

%-- Principal point:
cc = [ 638.499973934190850 ; 524.785925710220340 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.092502384223781 ; 0.187660352968980 ; -0.001096320242046 ; -0.000856105482605 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 7.189955633926862 ; 7.157732334774011 ];

%-- Principal point uncertainty:
cc_error = [ 6.528563816294828 ; 5.444223462802594 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006918049142312 ; 0.019526283741335 ; 0.000874526593286 ; 0.000905332658419 ; 0.000000000000000 ];

%-- Image size:
nx = 1392;
ny = 1040;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 15;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.196473e+000 ; 2.185199e+000 ; -1.271063e-001 ];
Tc_1  = [ -8.015144e+000 ; -3.434257e+002 ; 1.680528e+003 ];
omc_error_1 = [ 2.355885e-003 ; 2.873513e-003 ; 5.744011e-003 ];
Tc_error_1  = [ 6.190293e+000 ; 5.073748e+000 ; 6.836397e+000 ];

%-- Image #2:
omc_2 = [ 2.206811e+000 ; 2.197457e+000 ; -1.557103e-001 ];
Tc_2  = [ 7.337491e+000 ; -3.453850e+002 ; 1.551485e+003 ];
omc_error_2 = [ 2.291953e-003 ; 2.840771e-003 ; 5.759618e-003 ];
Tc_error_2  = [ 5.727470e+000 ; 4.682393e+000 ; 6.332141e+000 ];

%-- Image #3:
omc_3 = [ 2.199797e+000 ; 2.187956e+000 ; -1.402039e-001 ];
Tc_3  = [ -2.345689e+001 ; -3.408321e+002 ; 1.841380e+003 ];
omc_error_3 = [ 2.430534e-003 ; 2.956578e-003 ; 5.885414e-003 ];
Tc_error_3  = [ 6.769166e+000 ; 5.556846e+000 ; 7.467349e+000 ];

%-- Image #4:
omc_4 = [ 2.097113e+000 ; 2.071803e+000 ; 9.685633e-002 ];
Tc_4  = [ -6.631667e+001 ; -3.413510e+002 ; 1.782116e+003 ];
omc_error_4 = [ 2.459288e-003 ; 2.718386e-003 ; 5.038908e-003 ];
Tc_error_4  = [ 6.560903e+000 ; 5.406966e+000 ; 7.209433e+000 ];

%-- Image #5:
omc_5 = [ 1.909169e+000 ; 1.863616e+000 ; 4.306042e-001 ];
Tc_5  = [ -5.693209e+001 ; -3.426858e+002 ; 1.687534e+003 ];
omc_error_5 = [ 2.717486e-003 ; 2.585346e-003 ; 4.468594e-003 ];
Tc_error_5  = [ 6.223697e+000 ; 5.133868e+000 ; 6.941048e+000 ];

%-- Image #6:
omc_6 = [ -1.913355e+000 ; -1.938101e+000 ; 6.525036e-001 ];
Tc_6  = [ 1.135690e+002 ; -3.417236e+002 ; 1.837298e+003 ];
omc_error_6 = [ 3.047174e-003 ; 2.122544e-003 ; 5.190924e-003 ];
Tc_error_6  = [ 6.764419e+000 ; 5.596070e+000 ; 6.907836e+000 ];

%-- Image #7:
omc_7 = [ -2.092763e+000 ; -2.104893e+000 ; 4.073768e-001 ];
Tc_7  = [ 1.162204e+002 ; -3.421519e+002 ; 1.825537e+003 ];
omc_error_7 = [ 3.015636e-003 ; 2.261802e-003 ; 5.754104e-003 ];
Tc_error_7  = [ 6.718774e+000 ; 5.524364e+000 ; 7.236147e+000 ];

%-- Image #8:
omc_8 = [ 2.043411e+000 ; 2.050712e+000 ; -4.463154e-001 ];
Tc_8  = [ -1.490028e+001 ; -3.364159e+002 ; 1.764184e+003 ];
omc_error_8 = [ 1.890024e-003 ; 2.871141e-003 ; 5.175716e-003 ];
Tc_error_8  = [ 6.494576e+000 ; 5.315882e+000 ; 7.056292e+000 ];

%-- Image #9:
omc_9 = [ -2.174238e+000 ; -2.170520e+000 ; 1.799057e-002 ];
Tc_9  = [ -7.266755e+000 ; -3.451083e+002 ; 1.602184e+003 ];
omc_error_9 = [ 3.239624e-003 ; 3.026468e-003 ; 6.592664e-003 ];
Tc_error_9  = [ 5.902935e+000 ; 4.857214e+000 ; 6.485837e+000 ];

%-- Image #10:
omc_10 = [ -1.778011e+000 ; -1.986774e+000 ; 1.033049e+000 ];
Tc_10  = [ 3.219623e+002 ; -3.399157e+002 ; 1.760262e+003 ];
omc_error_10 = [ 3.486566e-003 ; 1.892923e-003 ; 4.715457e-003 ];
Tc_error_10  = [ 6.510870e+000 ; 5.394425e+000 ; 6.822313e+000 ];

%-- Image #11:
omc_11 = [ -1.924978e+000 ; -1.898384e+000 ; 4.886939e-001 ];
Tc_11  = [ 1.972538e+002 ; -3.463970e+002 ; 1.589100e+003 ];
omc_error_11 = [ 2.715826e-003 ; 2.157525e-003 ; 5.373845e-003 ];
Tc_error_11  = [ 5.876474e+000 ; 4.853683e+000 ; 6.212139e+000 ];

%-- Image #12:
omc_12 = [ -2.093201e+000 ; -2.119173e+000 ; 4.590054e-001 ];
Tc_12  = [ 3.181743e+002 ; -3.444333e+002 ; 1.735488e+003 ];
omc_error_12 = [ 2.978667e-003 ; 2.147861e-003 ; 5.644335e-003 ];
Tc_error_12  = [ 6.391070e+000 ; 5.278544e+000 ; 7.422674e+000 ];

%-- Image #13:
omc_13 = [ 1.927015e+000 ; 1.792368e+000 ; 4.645703e-004 ];
Tc_13  = [ -2.776081e+002 ; -3.347271e+002 ; 1.708355e+003 ];
omc_error_13 = [ 2.187655e-003 ; 2.776627e-003 ; 4.425493e-003 ];
Tc_error_13  = [ 6.307064e+000 ; 5.178419e+000 ; 7.173309e+000 ];

%-- Image #14:
omc_14 = [ 2.083772e+000 ; 2.044939e+000 ; 7.785406e-002 ];
Tc_14  = [ -1.839435e+002 ; -3.428606e+002 ; 1.579792e+003 ];
omc_error_14 = [ 2.288391e-003 ; 2.781966e-003 ; 4.818879e-003 ];
Tc_error_14  = [ 5.837862e+000 ; 4.786979e+000 ; 6.524230e+000 ];

%-- Image #15:
omc_15 = [ 2.082733e+000 ; 2.095777e+000 ; 3.209964e-001 ];
Tc_15  = [ -1.669440e+002 ; -3.431872e+002 ; 1.601273e+003 ];
omc_error_15 = [ 2.636709e-003 ; 2.666475e-003 ; 4.817487e-003 ];
Tc_error_15  = [ 5.934941e+000 ; 4.892157e+000 ; 6.497058e+000 ];

