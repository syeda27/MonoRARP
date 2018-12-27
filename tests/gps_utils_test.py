
import numpy as np
import sys

sys.path.append("..")
sys.path.append("../driver_risk_utils/")
import gps_utils

GPS_INTERFACE = gps_utils.GPS_Interface("logs_gps/gps_attempt2.log",
        readings_for_speed=2,
        gps_format='NMEA')

new_file = GPS_INTERFACE.copy_file()
print(new_file)
assert new_file == "logs_gps/gps_attempt2_last_100.log"

with open(new_file, "r") as f:
    lines = f.readlines()
    print(len(lines))
    assert len(lines) == 100

    print(lines[-1])
    assert lines[-1] == "gpsd:PROG: GPRMC ends a reporting cycle.\n"

    print(lines[0])
    assert lines[0] == "gpsd:IO: <= GPS: $GPGSV,3,2,12,24,17,059,37,18,16,310,29,04,00,000,24,14,41,198,19*7E\n"

last_reading_line = GPS_INTERFACE.get_last_reading_line(new_file)
assert last_reading_line == "gpsd:IO: <= GPS: $GPRMC,160347.000,A,3745.4843,N,12223.8672,W,0.00,26.10,271218,,,A*44\n"

last_reading_line = GPS_INTERFACE.get_last_valid_reading_line(new_file)
assert last_reading_line == "gpsd:IO: <= GPS: $GPRMC,160347.000,A,3745.4843,N,12223.8672,W,0.00,26.10,271218,,,A*44\n"


(time, lat, lon) = GPS_INTERFACE.parse_line(last_reading_line)
print(time)
print(lat)
print(lon)
#160347.000
#('37', '45.4843', 'N')
#('122', '23.8672', 'W')

assert time == "160347.000"
assert lat[0] == "37"
assert lat[1] == "45.4843"
assert lat[2] == "N"
assert lon[0] == "122"
assert lon[1] == "23.8672"
assert lon[2] == "W"

# now test speed
GPS_INTERFACE = gps_utils.GPS_Interface("logs_gps/gps_attempt2_first.log",
        readings_for_speed=2,
        gps_format='NMEA')
speed = GPS_INTERFACE.get_reading(True)
print(speed)
GPS_INTERFACE.change_file("logs_gps/gps_attempt2.log")
print(GPS_INTERFACE.last_readings)
speed = GPS_INTERFACE.get_reading(True)
print(speed)


print("GPS Utils test succeeded!")
