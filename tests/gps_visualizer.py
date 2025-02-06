import rerun as rr
import serial
from pyubx2 import UBXReader
from cobs import cobs
import zlib
import ctypes
import sys
import time


class BaseStation(ctypes.Structure):
    _fields_ = [
        ("gps", ctypes.c_char*100),
        ("error_mssg_flag", ctypes.c_uint16),
        ("crc", ctypes.c_uint32),
    ]


rr.init("GPS DATA", spawn=True)
rr.connect_tcp("192.168.80.11:9876")


def chmsg(error_mssg_flag):
    mssg = ""
    if error_mssg_flag & 0x0001 == 0x0001:
        mssg = mssg+"Error: Corrupt SBUS Packet"
    if error_mssg_flag & 0x0002 == 0x0002:
        mssg = mssg+"Error: COBS Decode Failed"
    if error_mssg_flag & 0x0004 == 0x0004:
        mssg = mssg+"Error: Message from Latte Panda is corrupt"
    if error_mssg_flag & 0x0008 == 0x0008:
        mssg = mssg+"Error: COBS Encoding Failed"
    if error_mssg_flag & 0x0010 == 0x0010:
        mssg = mssg+"Error: Unable to write PWM pulse to Left Motor"
    if error_mssg_flag & 0x0020 == 0x0020:
        mssg = mssg+"Error: Unable to write PWM pulse to Right Motor"
    if error_mssg_flag & 0x0040 == 0x0040:
        mssg = mssg+"Error: Unable to write to Gripper/Bio-Arm"
    if error_mssg_flag & 0x0080 == 0x0080:
        mssg = mssg+"Error: Unable to write to Tilt Servo"
    if error_mssg_flag & 0x0100 == 0x0100:
        mssg = mssg+"Error: Unable to write to Pan Servo"
    if error_mssg_flag & 0x0200 == 0x0200:
        mssg = mssg+"Error: Unable to write to Linear Actuator 1"
    if error_mssg_flag & 0x0400 == 0x0400:
        mssg = mssg+"Error: Unable to write to Linear Actuator 2"
    if error_mssg_flag & 0x0800 == 0x0800:
        mssg = mssg+"Error: Unable to write to to Mini-Acc/ABox"
    if error_mssg_flag & 0x1000 == 0x1000:
        mssg = mssg+"Error: No data from imu_lower_joint"
    if error_mssg_flag & 0x2000 == 0x2000:
        mssg = mssg+"Error: No data from imu_upper_joint"
    if error_mssg_flag & 0x4000 == 0x4000:
        mssg = mssg+"Error: No data from imu_pitch_roll"
    if error_mssg_flag & 0x8000 == 0x8000:
        mssg = mssg+"Error: No data from Magnetometer Turn Table"
    if error_mssg_flag == 0x0000:
        mssg = mssg+"No errors detected"
    return mssg


def process(encoded_msg, ser):
    if (encoded_msg[-1] != 0):
        print("COBS ERROR: mssg does not have end byte")
        return
    try:
        # decoding mssg
        decoded_msg = cobs.decode(encoded_msg[:-2])

        base_station_mssg = ctypes.cast(
            decoded_msg, ctypes.POINTER(BaseStation))

        # checking crc
        crc = base_station_mssg.contents.crc
        valid_crc = zlib.crc32(decoded_msg[:len(decoded_msg)-4])
        if (crc != valid_crc):
            print("ERROR: Mssg is corrupt crc did not match")
            return

        # decoding gps
        gps_msg = UBXReader.parse(decoded_msg[:100])
        if gps_msg.identity == "NAV-PVT":
            lat = gps_msg.lat
            lon = gps_msg.lon
            height = gps_msg.hMSL
            heading = gps_msg.headMot
            numSV = gps_msg.numSV
        else:
            print(f"Other message received: {gps_msg.identity}")

        # checking error mssg
        error_mssg = chmsg(base_station_mssg.contents.error_mssg_flag)

        return {
            "latitude": lat,
            "longitude": lon,
            "height": height,
            "heading": heading,
            "numSV": numSV,
            "msg_status": error_mssg,
        }
    except Exception as e:
        print(f"Error while processing data:{e}")


def main():
    port = sys.argv[1]
    ser = serial.Serial(port, baudrate=9600, timeout=1, exclusive=False)
    loc_data = list()
    while True:
        try:
            raw_data = ser.read(110)
            if raw_data:
                raw_data_with = raw_data + b'\x00'
                result = process(raw_data_with, ser)
            if result:
                print("result is : ", result)
                to_save = input("Save Data (yes/NO): ")
                # log geo data on map
                loc_data.append([result["latitude"], result["longitude"]])
                rr.log("geo_points", rr.GeoLineStrings(lat_lon=loc_data),
                       radii=rr.Radius.ui_points(2.0), colors=[0, 0, 255])
                # log heading
                rr.log("heading", rr.TextLog(
                    result["heading"], level=rr.TextLogLevel))
                # log data to save
                if (to_save == "yes" or to_save == "y"):
                    lat = result["latitude"]
                    lon = result["longitude"]
                    rr.log("object_loc",
                           rr.TextLog(f"lat:{lat}, lon:{lon}",
                                      level=rr.TextLogLevel))
        except Exception as e:
            print(f"error reading:{e}")

        time.sleep(2)


if __name__ == "__main__":
    main()
