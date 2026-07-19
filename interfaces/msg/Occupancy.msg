# World-frame anchored occupancy grid (plan §3.3, evolved from RX24).
std_msgs/Header header
float64[2] origin      # world-frame anchor: lat, lon of first RTK fix
float64[3] position    # boat x [m], y [m] (world frame), heading [rad]
float32 cell_size      # meters per cell

# Cell aging (diagram: "cell decay rate: tau = Xs"). v(t) = v0*exp(-dt/tau).
# Decay applies ONLY to SOURCE_PERCEPTION cells; SOURCE_COMMS cells persist
# until an explicit All Clear (Mission 4 requirement).
float32 decay_tau      # seconds; <= 0 disables decay entirely

Grid grid
