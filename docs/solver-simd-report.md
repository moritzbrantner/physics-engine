
## native: µs/tick (best of a round), and paired ratio to native-scalar

| workload | config | rounds | median µs | min µs | spread (p90/p10) | ratio median | ratio p10 | ratio p90 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| rubble-wake | native-scalar | 15 | 30.93 | 30.56 | 1.395 | 1.000 | 1.000 | 1.000 |
| rubble-wake | native-hot | 15 | 34.33 | 34.09 | 1.071 | 1.111 | 1.011 | 1.118 |
| rubble-wake | native-manifold | 15 | 41.00 | 40.84 | 1.623 | 1.329 | 1.298 | 1.377 |
| rubble-wake | native-colored | 15 | 34.03 | 33.31 | 1.431 | 1.102 | 0.837 | 1.522 |
| debris | native-scalar | 15 | 48.63 | 48.23 | 1.330 | 1.000 | 1.000 | 1.000 |
| debris | native-hot | 15 | 51.54 | 51.18 | 1.381 | 1.063 | 1.043 | 1.136 |
| debris | native-manifold | 15 | 58.21 | 57.86 | 1.615 | 1.200 | 1.183 | 1.321 |
| debris | native-colored | 15 | 57.04 | 56.31 | 1.413 | 1.166 | 0.920 | 1.628 |
| debris-town-tumble | native-scalar | 15 | 19.15 | 18.84 | 1.461 | 1.000 | 1.000 | 1.000 |
| debris-town-tumble | native-hot | 15 | 20.96 | 20.91 | 1.442 | 1.100 | 1.079 | 1.139 |
| debris-town-tumble | native-manifold | 15 | 78.87 | 77.57 | 1.739 | 4.139 | 4.016 | 4.311 |
| debris-town-tumble | native-colored | 15 | 23.64 | 23.17 | 1.460 | 1.231 | 0.911 | 1.740 |
| mmorpg-tick | native-scalar | 15 | 97.54 | 95.09 | 1.564 | 1.000 | 1.000 | 1.000 |
| mmorpg-tick | native-hot | 15 | 96.43 | 95.33 | 1.690 | 0.999 | 0.978 | 1.090 |
| mmorpg-tick | native-manifold | 15 | 96.08 | 95.58 | 1.412 | 0.989 | 0.979 | 1.051 |
| mmorpg-tick | native-colored | 15 | 96.80 | 95.10 | 1.576 | 1.000 | 0.710 | 1.544 |
| crowd | native-scalar | 15 | 62.57 | 61.98 | 1.370 | 1.000 | 1.000 | 1.000 |
| crowd | native-hot | 15 | 63.17 | 62.61 | 1.086 | 1.012 | 0.983 | 1.038 |
| crowd | native-manifold | 15 | 62.85 | 62.36 | 1.060 | 1.004 | 0.795 | 1.018 |
| crowd | native-colored | 15 | 62.47 | 61.89 | 1.504 | 1.000 | 0.779 | 1.460 |
| pile-16 | native-scalar | 15 | 32.25 | 32.05 | 1.266 | 1.000 | 1.000 | 1.000 |
| pile-16 | native-hot | 15 | 38.46 | 38.34 | 1.279 | 1.194 | 1.156 | 1.219 |
| pile-16 | native-manifold | 15 | 31.93 | 31.52 | 1.056 | 0.987 | 0.824 | 0.995 |
| pile-16 | native-colored | 15 | 35.68 | 34.96 | 1.395 | 1.101 | 0.896 | 1.516 |
| pile-128 | native-scalar | 15 | 315.46 | 314.09 | 1.330 | 1.000 | 1.000 | 1.000 |
| pile-128 | native-hot | 15 | 368.36 | 367.45 | 1.085 | 1.166 | 1.141 | 1.182 |
| pile-128 | native-manifold | 15 | 323.00 | 319.83 | 1.055 | 1.017 | 0.866 | 1.031 |
| pile-128 | native-colored | 15 | 333.49 | 329.34 | 1.394 | 1.055 | 0.832 | 1.464 |
| sparse-4 | native-scalar | 15 | 11.03 | 10.94 | 1.402 | 1.000 | 1.000 | 1.000 |
| sparse-4 | native-hot | 15 | 12.61 | 12.52 | 1.436 | 1.145 | 1.131 | 1.195 |
| sparse-4 | native-manifold | 15 | 10.58 | 10.50 | 1.073 | 0.961 | 0.786 | 0.971 |
| sparse-4 | native-colored | 15 | 12.50 | 12.36 | 1.508 | 1.137 | 0.859 | 1.701 |
| pile-1024 | native-scalar | 15 | 4548.24 | 4481.21 | 1.435 | 1.000 | 1.000 | 1.000 |
| pile-1024 | native-hot | 15 | 4938.90 | 4901.99 | 1.358 | 1.094 | 1.021 | 1.139 |
| pile-1024 | native-manifold | 15 | 5152.44 | 5072.69 | 1.131 | 1.129 | 0.902 | 1.163 |
| pile-1024 | native-colored | 15 | 4909.22 | 4822.92 | 1.496 | 1.079 | 0.804 | 1.565 |
| pile-4096 | native-scalar | 15 | 29138.76 | 28329.88 | 1.309 | 1.000 | 1.000 | 1.000 |
| pile-4096 | native-hot | 15 | 30894.65 | 30338.30 | 1.642 | 1.072 | 0.974 | 1.214 |
| pile-4096 | native-manifold | 15 | 30373.34 | 29963.36 | 1.074 | 1.047 | 0.862 | 1.065 |
| pile-4096 | native-colored | 15 | 30532.60 | 29710.94 | 1.982 | 1.053 | 0.981 | 1.388 |

## wasm: µs/tick (best of a round), and paired ratio to wasm-nosimd-scalar

| workload | config | rounds | median µs | min µs | spread (p90/p10) | ratio median | ratio p10 | ratio p90 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| rubble-wake | wasm-nosimd-scalar | 10 | 47.81 | 47.31 | 1.120 | 1.000 | 1.000 | 1.000 |
| rubble-wake | wasm-simd-scalar | 10 | 47.39 | 46.90 | 1.925 | 0.994 | 0.971 | 1.095 |
| rubble-wake | wasm-nosimd-hot | 10 | 51.82 | 51.16 | 1.018 | 1.077 | 1.007 | 1.094 |
| rubble-wake | wasm-simd-hot | 10 | 52.53 | 52.03 | 1.032 | 1.102 | 1.015 | 1.127 |
| rubble-wake | wasm-nosimd-manifold | 10 | 79.56 | 78.85 | 1.038 | 1.662 | 1.554 | 1.711 |
| rubble-wake | wasm-simd-manifold | 10 | 68.44 | 67.90 | 1.032 | 1.429 | 1.336 | 1.462 |
| rubble-wake | wasm-nosimd-colored | 10 | 71.28 | 70.89 | 1.017 | 1.491 | 1.403 | 1.509 |
| rubble-wake | wasm-simd-colored | 10 | 59.16 | 58.30 | 1.066 | 1.231 | 1.156 | 1.255 |
| debris | wasm-nosimd-scalar | 10 | 69.21 | 68.78 | 1.140 | 1.000 | 1.000 | 1.000 |
| debris | wasm-simd-scalar | 10 | 68.91 | 68.17 | 1.875 | 0.995 | 0.978 | 1.082 |
| debris | wasm-nosimd-hot | 10 | 73.46 | 72.82 | 1.013 | 1.061 | 0.965 | 1.070 |
| debris | wasm-simd-hot | 10 | 72.78 | 72.42 | 1.021 | 1.056 | 0.959 | 1.067 |
| debris | wasm-nosimd-manifold | 10 | 92.85 | 91.38 | 1.072 | 1.337 | 1.266 | 1.366 |
| debris | wasm-simd-manifold | 10 | 83.19 | 82.46 | 1.009 | 1.196 | 1.091 | 1.210 |
| debris | wasm-nosimd-colored | 10 | 90.64 | 90.06 | 1.015 | 1.310 | 1.189 | 1.328 |
| debris | wasm-simd-colored | 10 | 81.35 | 80.51 | 1.026 | 1.170 | 1.073 | 1.182 |
| debris-town-tumble | wasm-nosimd-scalar | 10 | 28.03 | 27.75 | 1.164 | 1.000 | 1.000 | 1.000 |
| debris-town-tumble | wasm-simd-scalar | 10 | 27.21 | 26.97 | 1.811 | 0.971 | 0.923 | 1.073 |
| debris-town-tumble | wasm-nosimd-hot | 10 | 30.92 | 30.64 | 1.012 | 1.103 | 0.997 | 1.115 |
| debris-town-tumble | wasm-simd-hot | 10 | 30.20 | 29.95 | 1.025 | 1.082 | 0.968 | 1.098 |
| debris-town-tumble | wasm-nosimd-manifold | 10 | 116.35 | 115.27 | 1.024 | 4.142 | 3.785 | 4.214 |
| debris-town-tumble | wasm-simd-manifold | 10 | 106.40 | 105.34 | 1.019 | 3.790 | 3.424 | 3.854 |
| debris-town-tumble | wasm-nosimd-colored | 10 | 37.27 | 37.02 | 1.022 | 1.330 | 1.196 | 1.363 |
| debris-town-tumble | wasm-simd-colored | 10 | 34.14 | 33.81 | 1.015 | 1.217 | 1.103 | 1.226 |
| mmorpg-tick | wasm-nosimd-scalar | 10 | 147.93 | 145.59 | 1.203 | 1.000 | 1.000 | 1.000 |
| mmorpg-tick | wasm-simd-scalar | 10 | 144.31 | 142.13 | 1.183 | 0.979 | 0.897 | 1.119 |
| mmorpg-tick | wasm-nosimd-hot | 10 | 148.23 | 145.81 | 1.022 | 1.003 | 0.927 | 1.025 |
| mmorpg-tick | wasm-simd-hot | 10 | 144.97 | 143.94 | 1.022 | 0.981 | 0.907 | 1.007 |
| mmorpg-tick | wasm-nosimd-manifold | 10 | 147.22 | 144.97 | 1.029 | 1.001 | 0.912 | 1.021 |
| mmorpg-tick | wasm-simd-manifold | 10 | 143.56 | 142.10 | 1.019 | 0.968 | 0.903 | 0.987 |
| mmorpg-tick | wasm-nosimd-colored | 10 | 148.97 | 145.45 | 1.030 | 1.012 | 0.926 | 1.027 |
| mmorpg-tick | wasm-simd-colored | 10 | 145.53 | 143.84 | 1.044 | 0.984 | 0.908 | 1.007 |
| crowd | wasm-nosimd-scalar | 10 | 93.07 | 91.96 | 1.152 | 1.000 | 1.000 | 1.000 |
| crowd | wasm-simd-scalar | 10 | 92.52 | 91.20 | 1.123 | 0.991 | 0.925 | 1.101 |
| crowd | wasm-nosimd-hot | 10 | 92.64 | 91.85 | 1.043 | 0.994 | 0.924 | 1.042 |
| crowd | wasm-simd-hot | 10 | 93.46 | 92.66 | 1.022 | 1.001 | 0.932 | 1.017 |
| crowd | wasm-nosimd-manifold | 10 | 92.64 | 91.90 | 1.028 | 0.997 | 0.930 | 1.021 |
| crowd | wasm-simd-manifold | 10 | 92.74 | 91.76 | 1.017 | 0.996 | 0.923 | 1.007 |
| crowd | wasm-nosimd-colored | 10 | 93.07 | 91.87 | 1.024 | 1.001 | 0.935 | 1.014 |
| crowd | wasm-simd-colored | 10 | 94.13 | 92.80 | 1.024 | 1.007 | 0.943 | 1.021 |
| pile-16 | wasm-nosimd-scalar | 10 | 33.77 | 33.53 | 1.238 | 1.000 | 1.000 | 1.000 |
| pile-16 | wasm-simd-scalar | 10 | 33.52 | 33.28 | 1.115 | 0.992 | 0.920 | 1.100 |
| pile-16 | wasm-nosimd-hot | 10 | 38.64 | 38.08 | 1.060 | 1.141 | 1.049 | 1.201 |
| pile-16 | wasm-simd-hot | 10 | 41.39 | 40.86 | 1.019 | 1.228 | 1.126 | 1.244 |
| pile-16 | wasm-nosimd-manifold | 10 | 51.27 | 50.57 | 1.054 | 1.516 | 1.385 | 1.580 |
| pile-16 | wasm-simd-manifold | 10 | 41.67 | 41.07 | 1.022 | 1.229 | 1.136 | 1.257 |
| pile-16 | wasm-nosimd-colored | 10 | 57.45 | 56.88 | 1.016 | 1.707 | 1.557 | 1.719 |
| pile-16 | wasm-simd-colored | 10 | 45.16 | 44.14 | 1.031 | 1.337 | 1.216 | 1.359 |
| pile-128 | wasm-nosimd-scalar | 10 | 340.48 | 338.08 | 1.111 | 1.000 | 1.000 | 1.000 |
| pile-128 | wasm-simd-scalar | 10 | 336.66 | 335.18 | 1.097 | 0.990 | 0.931 | 1.084 |
| pile-128 | wasm-nosimd-hot | 10 | 381.39 | 379.36 | 1.045 | 1.127 | 1.051 | 1.168 |
| pile-128 | wasm-simd-hot | 10 | 400.59 | 397.57 | 1.017 | 1.178 | 1.102 | 1.200 |
| pile-128 | wasm-nosimd-manifold | 10 | 500.53 | 496.63 | 1.034 | 1.478 | 1.387 | 1.515 |
| pile-128 | wasm-simd-manifold | 10 | 418.14 | 413.73 | 1.020 | 1.231 | 1.147 | 1.247 |
| pile-128 | wasm-nosimd-colored | 10 | 533.41 | 530.45 | 1.028 | 1.566 | 1.470 | 1.611 |
| pile-128 | wasm-simd-colored | 10 | 419.27 | 415.15 | 1.019 | 1.232 | 1.151 | 1.245 |
| sparse-4 | wasm-nosimd-scalar | 10 | 16.67 | 16.55 | 1.109 | 1.000 | 1.000 | 1.000 |
| sparse-4 | wasm-simd-scalar | 10 | 16.60 | 16.50 | 1.111 | 0.996 | 0.939 | 1.107 |
| sparse-4 | wasm-nosimd-hot | 10 | 17.87 | 17.82 | 1.112 | 1.074 | 1.010 | 1.188 |
| sparse-4 | wasm-simd-hot | 10 | 18.60 | 18.50 | 1.009 | 1.116 | 1.055 | 1.121 |
| sparse-4 | wasm-nosimd-manifold | 10 | 20.39 | 20.22 | 1.051 | 1.222 | 1.146 | 1.277 |
| sparse-4 | wasm-simd-manifold | 10 | 18.06 | 17.97 | 1.011 | 1.084 | 1.020 | 1.088 |
| sparse-4 | wasm-nosimd-colored | 10 | 21.36 | 21.28 | 1.084 | 1.284 | 1.206 | 1.385 |
| sparse-4 | wasm-simd-colored | 10 | 19.62 | 19.53 | 1.024 | 1.177 | 1.112 | 1.199 |
| pile-1024 | wasm-nosimd-scalar | 10 | 5705.51 | 5563.57 | 1.368 | 1.000 | 1.000 | 1.000 |
| pile-1024 | wasm-simd-scalar | 10 | 5579.64 | 5527.59 | 1.145 | 0.984 | 0.863 | 1.104 |
| pile-1024 | wasm-nosimd-hot | 10 | 6029.07 | 5907.43 | 1.085 | 1.063 | 0.937 | 1.122 |
| pile-1024 | wasm-simd-hot | 10 | 6031.55 | 5963.06 | 1.042 | 1.072 | 0.947 | 1.097 |
| pile-1024 | wasm-nosimd-manifold | 10 | 7561.92 | 7393.51 | 1.061 | 1.323 | 1.152 | 1.405 |
| pile-1024 | wasm-simd-manifold | 10 | 6577.94 | 6502.10 | 1.031 | 1.166 | 1.047 | 1.180 |
| pile-1024 | wasm-nosimd-colored | 10 | 7453.58 | 7318.19 | 1.082 | 1.305 | 1.146 | 1.396 |
| pile-1024 | wasm-simd-colored | 10 | 6266.28 | 6136.11 | 1.194 | 1.096 | 0.985 | 1.136 |
| pile-4096 | wasm-nosimd-scalar | 10 | 35804.49 | 35235.11 | 1.194 | 1.000 | 1.000 | 1.000 |
| pile-4096 | wasm-simd-scalar | 10 | 35132.06 | 34578.81 | 1.160 | 0.987 | 0.885 | 1.128 |
| pile-4096 | wasm-nosimd-hot | 10 | 37581.49 | 36395.74 | 1.297 | 1.040 | 0.960 | 1.301 |
| pile-4096 | wasm-simd-hot | 10 | 36864.18 | 36485.92 | 1.037 | 1.029 | 0.946 | 1.056 |
| pile-4096 | wasm-nosimd-manifold | 10 | 42835.23 | 41334.09 | 1.322 | 1.182 | 1.067 | 1.551 |
| pile-4096 | wasm-simd-manifold | 10 | 38240.67 | 37736.64 | 1.037 | 1.070 | 0.984 | 1.104 |
| pile-4096 | wasm-nosimd-colored | 10 | 43555.92 | 42705.04 | 1.518 | 1.207 | 1.099 | 1.827 |
| pile-4096 | wasm-simd-colored | 10 | 38430.06 | 37419.54 | 1.059 | 1.065 | 0.985 | 1.105 |

## phases: µs/tick, median over rounds, from the phase-timing builds

| workload | config | rounds | rows | solve | of which prepare |
| --- | --- | ---: | ---: | ---: | ---: |
| rubble-wake | native-colored-phase | 15 | 4.30 | 12.87 | 4.87 |
| rubble-wake | native-hot-phase | 15 | 4.19 | 13.17 | 1.26 |
| rubble-wake | native-manifold-phase | 15 | 5.39 | 14.94 | 7.29 |
| rubble-wake | native-scalar-phase | 15 | 4.21 | 9.50 | 0.00 |
| rubble-wake | wasm-nosimd-colored-phase | 10 | 7.58 | 34.21 | 10.59 |
| rubble-wake | wasm-nosimd-hot-phase | 10 | 6.79 | 16.28 | 2.83 |
| rubble-wake | wasm-nosimd-manifold-phase | 10 | 9.11 | 35.35 | 14.26 |
| rubble-wake | wasm-nosimd-scalar-phase | 10 | 6.77 | 12.32 | 0.00 |
| rubble-wake | wasm-simd-colored-phase | 10 | 7.46 | 23.23 | 10.78 |
| rubble-wake | wasm-simd-hot-phase | 10 | 6.70 | 18.09 | 2.83 |
| rubble-wake | wasm-simd-manifold-phase | 10 | 8.89 | 25.99 | 14.22 |
| rubble-wake | wasm-simd-scalar-phase | 10 | 6.79 | 12.96 | 0.00 |
| debris | native-colored-phase | 15 | 7.41 | 15.08 | 6.01 |
| debris | native-hot-phase | 15 | 7.38 | 11.19 | 1.61 |
| debris | native-manifold-phase | 15 | 8.60 | 14.48 | 8.05 |
| debris | native-scalar-phase | 15 | 7.33 | 8.11 | 0.00 |
| debris | wasm-nosimd-colored-phase | 10 | 9.39 | 28.68 | 9.28 |
| debris | wasm-nosimd-hot-phase | 10 | 9.27 | 13.18 | 2.85 |
| debris | wasm-nosimd-manifold-phase | 10 | 10.99 | 27.23 | 11.51 |
| debris | wasm-nosimd-scalar-phase | 10 | 9.35 | 9.21 | 0.00 |
| debris | wasm-simd-colored-phase | 10 | 9.11 | 19.81 | 9.25 |
| debris | wasm-simd-hot-phase | 10 | 8.96 | 13.58 | 2.88 |
| debris | wasm-simd-manifold-phase | 10 | 10.60 | 19.31 | 11.59 |
| debris | wasm-simd-scalar-phase | 10 | 9.17 | 9.68 | 0.00 |
| debris-town-tumble | native-colored-phase | 15 | 4.65 | 7.92 | 5.28 |
| debris-town-tumble | native-hot-phase | 15 | 4.68 | 5.09 | 1.41 |
| debris-town-tumble | native-manifold-phase | 15 | 15.54 | 35.93 | 28.19 |
| debris-town-tumble | native-scalar-phase | 15 | 4.68 | 3.04 | 0.00 |
| debris-town-tumble | wasm-nosimd-colored-phase | 10 | 6.22 | 13.54 | 8.09 |
| debris-town-tumble | wasm-nosimd-hot-phase | 10 | 6.24 | 6.38 | 2.55 |
| debris-town-tumble | wasm-nosimd-manifold-phase | 10 | 20.24 | 57.14 | 40.25 |
| debris-town-tumble | wasm-nosimd-scalar-phase | 10 | 6.31 | 3.63 | 0.00 |
| debris-town-tumble | wasm-simd-colored-phase | 10 | 5.94 | 11.33 | 8.09 |
| debris-town-tumble | wasm-simd-hot-phase | 10 | 5.92 | 6.66 | 2.52 |
| debris-town-tumble | wasm-simd-manifold-phase | 10 | 19.06 | 50.86 | 40.35 |
| debris-town-tumble | wasm-simd-scalar-phase | 10 | 6.13 | 3.63 | 0.00 |
| mmorpg-tick | native-colored-phase | 15 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | native-hot-phase | 15 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | native-manifold-phase | 15 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | native-scalar-phase | 15 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-nosimd-colored-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-nosimd-hot-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-nosimd-manifold-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-nosimd-scalar-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-simd-colored-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-simd-hot-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-simd-manifold-phase | 10 | 0.00 | 0.00 | 0.00 |
| mmorpg-tick | wasm-simd-scalar-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | native-colored-phase | 15 | 0.00 | 0.00 | 0.00 |
| crowd | native-hot-phase | 15 | 0.00 | 0.00 | 0.00 |
| crowd | native-manifold-phase | 15 | 0.00 | 0.00 | 0.00 |
| crowd | native-scalar-phase | 15 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-nosimd-colored-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-nosimd-hot-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-nosimd-manifold-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-nosimd-scalar-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-simd-colored-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-simd-hot-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-simd-manifold-phase | 10 | 0.00 | 0.00 | 0.00 |
| crowd | wasm-simd-scalar-phase | 10 | 0.00 | 0.00 | 0.00 |
| pile-16 | native-colored-phase | 15 | 4.40 | 21.96 | 8.12 |
| pile-16 | native-hot-phase | 15 | 4.38 | 24.74 | 1.95 |
| pile-16 | native-manifold-phase | 15 | 5.46 | 16.21 | 9.40 |
| pile-16 | native-scalar-phase | 15 | 4.39 | 18.21 | 0.00 |
| pile-16 | wasm-nosimd-colored-phase | 10 | 5.32 | 40.11 | 12.65 |
| pile-16 | wasm-nosimd-hot-phase | 10 | 5.33 | 20.57 | 3.64 |
| pile-16 | wasm-nosimd-manifold-phase | 10 | 7.09 | 30.35 | 13.50 |
| pile-16 | wasm-nosimd-scalar-phase | 10 | 5.40 | 15.74 | 0.00 |
| pile-16 | wasm-simd-colored-phase | 10 | 5.11 | 28.21 | 12.69 |
| pile-16 | wasm-simd-hot-phase | 10 | 5.17 | 24.41 | 3.64 |
| pile-16 | wasm-simd-manifold-phase | 10 | 6.85 | 21.90 | 13.43 |
| pile-16 | wasm-simd-scalar-phase | 10 | 5.27 | 16.51 | 0.00 |
| pile-128 | native-colored-phase | 15 | 40.75 | 174.75 | 69.69 |
| pile-128 | native-hot-phase | 15 | 41.28 | 206.56 | 16.93 |
| pile-128 | native-manifold-phase | 15 | 50.18 | 134.24 | 78.71 |
| pile-128 | native-scalar-phase | 15 | 41.68 | 153.15 | 0.00 |
| pile-128 | wasm-nosimd-colored-phase | 10 | 50.06 | 333.66 | 104.61 |
| pile-128 | wasm-nosimd-hot-phase | 10 | 50.73 | 176.51 | 29.90 |
| pile-128 | wasm-nosimd-manifold-phase | 10 | 64.63 | 254.11 | 113.57 |
| pile-128 | wasm-nosimd-scalar-phase | 10 | 51.01 | 137.59 | 0.00 |
| pile-128 | wasm-simd-colored-phase | 10 | 49.82 | 225.57 | 106.34 |
| pile-128 | wasm-simd-hot-phase | 10 | 50.31 | 205.22 | 29.94 |
| pile-128 | wasm-simd-manifold-phase | 10 | 62.73 | 179.42 | 112.55 |
| pile-128 | wasm-simd-scalar-phase | 10 | 50.49 | 142.31 | 0.00 |
| sparse-4 | native-colored-phase | 15 | 1.01 | 6.18 | 1.69 |
| sparse-4 | native-hot-phase | 15 | 0.98 | 6.32 | 0.56 |
| sparse-4 | native-manifold-phase | 15 | 0.97 | 4.27 | 2.35 |
| sparse-4 | native-scalar-phase | 15 | 0.98 | 4.52 | 0.00 |
| sparse-4 | wasm-nosimd-colored-phase | 10 | 1.31 | 8.98 | 2.81 |
| sparse-4 | wasm-nosimd-hot-phase | 10 | 1.27 | 5.27 | 1.04 |
| sparse-4 | wasm-nosimd-manifold-phase | 10 | 1.31 | 7.94 | 3.47 |
| sparse-4 | wasm-nosimd-scalar-phase | 10 | 1.28 | 3.84 | 0.00 |
| sparse-4 | wasm-simd-colored-phase | 10 | 1.23 | 7.49 | 2.79 |
| sparse-4 | wasm-simd-hot-phase | 10 | 1.21 | 6.37 | 1.05 |
| sparse-4 | wasm-simd-manifold-phase | 10 | 1.22 | 5.82 | 3.40 |
| sparse-4 | wasm-simd-scalar-phase | 10 | 1.22 | 4.08 | 0.00 |
| pile-1024 | native-colored-phase | 15 | 615.72 | 1593.42 | 631.40 |
| pile-1024 | native-hot-phase | 15 | 597.16 | 1689.15 | 165.78 |
| pile-1024 | native-manifold-phase | 15 | 655.69 | 1418.59 | 728.49 |
| pile-1024 | native-scalar-phase | 15 | 600.37 | 1255.71 | 0.00 |
| pile-1024 | wasm-nosimd-colored-phase | 10 | 740.71 | 3066.61 | 937.35 |
| pile-1024 | wasm-nosimd-hot-phase | 10 | 723.25 | 1690.40 | 272.74 |
| pile-1024 | wasm-nosimd-manifold-phase | 10 | 818.23 | 2651.62 | 1064.70 |
| pile-1024 | wasm-nosimd-scalar-phase | 10 | 735.55 | 1367.01 | 0.00 |
| pile-1024 | wasm-simd-colored-phase | 10 | 734.51 | 1968.45 | 946.29 |
| pile-1024 | wasm-simd-hot-phase | 10 | 731.69 | 1875.61 | 278.72 |
| pile-1024 | wasm-simd-manifold-phase | 10 | 786.92 | 1851.00 | 1057.38 |
| pile-1024 | wasm-simd-scalar-phase | 10 | 727.45 | 1405.30 | 0.00 |
| pile-4096 | native-colored-phase | 15 | 3175.49 | 7477.75 | 3098.41 |
| pile-4096 | native-hot-phase | 15 | 3054.97 | 7627.25 | 847.28 |
| pile-4096 | native-manifold-phase | 15 | 3080.36 | 6398.09 | 3141.80 |
| pile-4096 | native-scalar-phase | 15 | 3060.51 | 5719.75 | 0.00 |
| pile-4096 | wasm-nosimd-colored-phase | 10 | 3670.99 | 14193.79 | 4542.25 |
| pile-4096 | wasm-nosimd-hot-phase | 10 | 3512.45 | 7653.98 | 1260.76 |
| pile-4096 | wasm-nosimd-manifold-phase | 10 | 3617.96 | 11476.61 | 4555.64 |
| pile-4096 | wasm-nosimd-scalar-phase | 10 | 3534.36 | 6311.06 | 0.00 |
| pile-4096 | wasm-simd-colored-phase | 10 | 3615.48 | 9092.65 | 4542.41 |
| pile-4096 | wasm-simd-hot-phase | 10 | 3534.39 | 8344.35 | 1289.89 |
| pile-4096 | wasm-simd-manifold-phase | 10 | 3512.22 | 8267.85 | 4499.89 |
| pile-4096 | wasm-simd-scalar-phase | 10 | 3561.54 | 6519.92 | 0.00 |
| wasm build (engine only) | bytes |
| --- | ---: |
| nosimd scalar | 1743457 |
| nosimd hot | 1741001 |
| nosimd manifold | 1755030 |
| nosimd colored | 1787581 |
| simd scalar | 1720858 |
| simd hot | 1714855 |
| simd manifold | 1727996 |
| simd colored | 1750108 |
