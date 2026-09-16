std_msgs/Header header
# Public controller state only: p_NED(3), v_NED(3), q_WB(4), omega_FRD(3).
float64[3] position_ned_m
float64[3] velocity_ned_mps
# Hamilton quaternion [w, x, y, z], rotating body FRD vectors into world NED.
float64[4] attitude_wb
float64[3] body_rates_frd_radps
# Row-major covariance for the public 13-dimensional state.
float64[169] covariance

