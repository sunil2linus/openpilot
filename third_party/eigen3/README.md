Eigen 3.4.0 headers (https://eigen.tuxfamily.org), from tag 3.4.0 (commit 3147391d946bb4b6c68edd901f2add6ac1f31f8c).
Vendored because StarPilot AGNOS 19.x no longer ships libeigen3-dev, so on-device builds
failed with "eigen3/Eigen/Dense file not found". Found via the existing "#third_party" CPPPATH
entry as <eigen3/Eigen/...>. Licensed MPL2 (see COPYING.MPL2).
