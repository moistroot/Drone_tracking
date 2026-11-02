# Drone_tracking
This project implements GCC-PHAT on an FPGA for real-time acoustic source localization. Multiple microphones capture synchronized audio; FFT-based cross-correlation with PHAT weighting estimates the time difference of arrival (TDOA), and geometric solving derives the source position. FPGA parallelism enables low-latency embedded processing.
