std_msgs/Header header

int32 current_sector          # index of the sector currently being scanned
float64 sector_center_deg     # bearing of the sector's centre
float64 sector_width_deg      # angular width of a sector
float64 max_range             # meters
SonarContact[] contacts       # targets detected within the current sector