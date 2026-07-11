string prn        # Satellite PRN number
uint16 week       # GPS week
float64 tow       # GPS time of week
uint16[] snr      # Signal strengths (was SNR in ROS1)
uint8[] lli       # Loss of lock indicators (was LLI)
string[] code     # Code indicators
float64[] l       # Carrier phase cycles (was L)
float64[] p       # Pseudoranges (was P)
float64[] d       # Dopplers (was D)
