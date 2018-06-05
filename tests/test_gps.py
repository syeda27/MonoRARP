
import numpy as np
import sys

sys.path.append("..")
import use_gps

GPS_INTERFACE = use_gps.gps_interface("gps_attempt2.log",
        readings_for_speed=2,
        gps_format='NMEA')

new_file = GPS_INTERFACE.copy_file()
print(new_file)
assert new_file == "gps_attempt2_last_100.log"

with open(new_file, "r") as f:
    lines = f.readlines()
    print(len(lines))
    assert len(lines) == 100 

    print(lines[-1])
    assert lines[-1] == "gpsd:PROG: GPRMC ends a reporting cycle.\n" 

    print(lines[0])
    assert lines[0] == "gpsd:IO: <= GPS: $GPGSA,M,3,17,28,19,13,30,15,01,07,18,,,,1.9,1.0,1.6*3E\n"
    
last_reading_line = GPS_INTERFACE.get_last_reading_line(new_file) 
assert last_reading_line == "gpsd:IO: <= GPS: $GPRMC,183423.000,A,3725.2744,N,12210.4042,W,0.00,264.67,030618,,,A*7A\n"

(time, lat, lon) = GPS_INTERFACE.parse_line(last_reading_line)
print(time)
print(lat)
print(lon)
assert time == "183423.000"
assert lat[0] == "37"
assert lat[1] == "25.2744"
assert lat[2] == "N"
assert lon[0] == "122"
assert lon[1] == "10.4042"
assert lon[2] == "W"

# now test speed
GPS_INTERFACE = use_gps.gps_interface("gps_attempt2_first.log",
        readings_for_speed=2,
        gps_format='NMEA')
speed = GPS_INTERFACE.get_reading(True)
print(speed)
GPS_INTERFACE.change_file("gps_attempt2.log")
print(GPS_INTERFACE.last_readings)
speed = GPS_INTERFACE.get_reading(True)
print(speed)
