import subprocess
import os
import numpy as np

# This file is intended not to start the gps but rather to read and parse
# a file that a separate process is already producing.
# For our use case, we need to get the time, longitude, latitude,
# and then convert two or more consequtive readings into speed.
# We assume it is called from another script and has filename passed in.


class GPS_Interface:
    last_readings = []

    def __init__(self, filename,
            readings_for_speed=2,
            gps_format='NMEA'):
        self.filename = filename
        self.n_readings = readings_for_speed
        self.format = gps_format

        if gps_format != "NMEA":
            print("Format {} is currently not supported.".format(gps_format))

    # mostly for test
    def change_file(self, new_name):
        self.filename = new_name

    # reading is (time, lat, lon)
    def add_reading(self, reading, verbose = False):
        if len(self.last_readings) > 0 and \
                reading[0] == self.last_readings[-1][0]:
            if verbose:
                print("Reading not added because no time change")
            return
        # do not add, if time is the same
        self.last_readings.append(reading)
        if verbose:
            print("Should have appended {}".format(reading))
            print(self.last_readings)
        self.last_readings = self.last_readings[-self.n_readings:]
        if verbose:
            print("Should have removed the first")
            print(self.last_readings)

    # returns speed
    def get_reading(self, verbose=False):
        # First step is copy the last 100 lines from the file to a new file.
        # The idea is to avoid concurrent read/write issues
        new_file = self.copy_file()
        line = self.get_last_reading_line(new_file)
        (time, lat, lon) = self.parse_line(line)
        if verbose:
            print("time: {}, Lat: {}, Lon: {}".format(time, " ".join(lat), " ".join(lon)))
        speed = self.calc_speed(time, lat, lon, verbose)

        # update class history
        self.add_reading((time, lat, lon), verbose=False)
        os.remove(new_file) # delete copied file when done

        return speed

    def copy_file(self, n_lines=100):
        file_name_components = self.filename.split(".")
        filename = ".".join(file_name_components[:-1]) # all except extension
        extension = file_name_components[-1]
        new_file = ".".join([filename + "_last_{}".format(n_lines),
                            extension])
        proc = subprocess.Popen(['tail', '-n', str(n_lines), self.filename],
                stdout=subprocess.PIPE)
        lines = [line.decode ("utf-8") for line in proc.stdout.readlines()]
        with open(new_file, "w") as f:
            f.writelines(lines)
        return new_file

    def get_last_reading_line(self, new_file):
        with open(new_file, "r") as f:
            for line in reversed(f.readlines()):
                if self.format == "NMEA":
                    if line.split(",")[0] == "gpsd:IO: <= GPS: $GPRMC":
                        return line
                else:
                    print("Unsupported format")
                    return ""

    # also checks validity
    def get_last_valid_reading_line(self, new_file):
        with open(new_file, "r") as f:
            for line in reversed(f.readlines()):
                if self.format == "NMEA":
                    if line.split(",")[0] == "gpsd:IO: <= GPS: $GPRMC":
                        if line.split(",")[2] == "A":
                            return line
                else:
                    print("Unsupported format")
                    return ""

    def parse_line(self, line, verbose=False):
        if line is None:
            if verbose:
                print("no line passed to parse_line()")
            return (-1,-1,-1)
        if self.format == "NMEA":
            splits = line.split(",")
            active = splits[2] == "A"
            if not active:
                if splits[2] == "V":
                    print("reading is void")
                    return (-1, -1, -1)
                else:
                    print("line is formatted incorrectly")
                    return (-1, -1, -1)
            time = splits[1]
            lat = splits[3]
            deg_lat = lat[:-7]
            minutes_lat = lat[len(deg_lat):]
            hem_lat = splits[4]
            lon = splits[5]
            deg_lon = lon[:-7]
            minutes_lon = lon[len(deg_lon):]
            hem_lon = splits[6]
            return (time, (deg_lat, minutes_lat, hem_lat),
                          (deg_lon, minutes_lon, hem_lon))
        else:
            print("Unsupported format")
            return (-1, -1, -1)

    def calc_speed(self, time, lat, lon, verbose=False):
        if len(self.last_readings) == 0:
            if verbose:
                print("No prior readings.")
            return 0
        time1, lat1, lon1 = self.last_readings[-1]
        dt = float(time) - float(time1)
        if dt == 0:
            if verbose:
                print("No time change in readings, aka no new reading")
            return 0
        distance = calc_distance(float("".join((lat1[0],lat1[1]))),
                                 float("".join((lon1[0],lon1[1]))),
                                 float("".join((lat[0],lat[1]))),
                                 float("".join((lon[0],lon[1]))))
        speed = distance / dt
        return speed

# use the Haversine formula to determine the "great-circle" distance
# between multiple longitude points
# Return meters
def calc_distance(lat1, lon1, lat2, lon2):
    rad_earth = 6378.137 # radius of earth in km
    lat1_ = np.deg2rad(lat1)
    lat2_ = np.deg2rad(lat2)
    a = np.power(np.sin((lat2_ - lat1_)/2.0), 2) + \
            np.cos(lat1_) * np.cos(lat2_) * \
            np.power(np.sin(np.deg2rad(lon2 - lon1)/2.0), 2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0-a))
    return rad_earth * 1000.0 * c
