string prn        # Satellite PRN number
uint16 week       # GPS week
uint8 sva         # SV accuracy (URA index)
uint16 code       # GPS: code on L2, GAL/BDS: data source
uint8 iode        # Issue of data, ephemeris
uint16 iodc       # Issue of data, clock
uint8 svh         # SV health
float64 toc       # Time of clock
float64 idot      # SV orbit parameter
float64 crs       # SV orbit parameter
float64 deln      # SV orbit parameter
float64 m0        # SV orbit parameter (was M0)
float64 cuc       # SV orbit parameter
float64 e         # SV orbit parameter
float64 cus       # SV orbit parameter
float64 a         # SV orbit parameter (was A)
float64 toes      # SV orbit parameter
float64 cic       # SV orbit parameter
float64 omg0      # SV orbit parameter (was OMG0)
float64 cis       # SV orbit parameter
float64 i0        # SV orbit parameter
float64 crc       # SV orbit parameter
float64 omg       # SV orbit parameter
float64 omgd      # SV orbit parameter (was OMGd)
float64[] tgd     # Group delay parameters
float64 f2        # SV clock parameter
float64 f1        # SV clock parameter
float64 f0        # SV clock parameter
