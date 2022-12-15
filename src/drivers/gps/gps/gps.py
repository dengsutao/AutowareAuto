# -*- coding:utf-8 -*-
import serial
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, TimeReference, NavSatStatus
from quaternions import Quaternion 
from gps.parser import parse_nmea_sentence

def check_nmea_checksum(nmea_sentence):
    split_sentence = nmea_sentence.split('*')
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA
        # data?
        return False
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    return ("%02X" % checksum) == transmitted_checksum.upper()

class GPS_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("port", "/dev/gps")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("debug", False)
        self.declare_parameter('~epe_quality0', 1000000)
        self.declare_parameter('~epe_quality1', 4.0)
        self.declare_parameter('~epe_quality2', 0.1)
        self.declare_parameter('~epe_quality4', 0.02)
        self.declare_parameter('~epe_quality5', 4.0)
        self.declare_parameter('~epe_quality9', 3.0)
        self.timeout = 0.5
        self.pub_gps = self.create_publisher(NavSatFix, "gps", 10)
        self.ser = self.init_serial()
        self.default_epe_quality0 = self.get_parameter('~epe_quality0').value
        self.default_epe_quality1 = self.get_parameter('~epe_quality1').value
        self.default_epe_quality2 = self.get_parameter('~epe_quality2').value
        self.default_epe_quality4 = self.get_parameter('~epe_quality4').value
        self.default_epe_quality5 = self.get_parameter('~epe_quality5').value
        self.default_epe_quality9 = self.get_parameter('~epe_quality9').value
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")
        self.valid_fix = False
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def init_serial(self):
        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.get_logger().info(f"port={port},baudrate={baudrate}")
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=self.timeout)
            if ser.is_open:
                self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().info("Serial port opening failure")
            exit(0)
        else:
            return ser

    def read_and_publish(self):
        data = self.ser.readline().strip().decode('utf-8')
        # print(data)
        if not check_nmea_checksum(data):
            # self.get_logger().warn("Received a sentence with an invalid checksum. " +
            #               "Sentence was: %s" % repr(data))
            return False
        
        parsed_sentence = parse_nmea_sentence(data)
        if not parsed_sentence:
            # self.get_logger().warn(
            #     "Failed to parse NMEA sentence. Sentence was: %s" %
            #     data)
            return False
        # print(parsed_sentence)
        current_time = self.get_clock().now().to_msg()
        frame_id = "gps"
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id

        if 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude if not math.isnan(latitude) else 0.0

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude if not math.isnan(longitude) else 0.0

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude if not math.isnan(altitude) else 0.0

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (
                2 * hdop * self.alt_std_dev) ** 2  # FIXME

            if self.get_parameter("debug").value:
                self.get_logger().info("lon,lat,alt-->"+str(current_fix.longitude)+','+str(current_fix.latitude)+','+str(current_fix.altitude))
            self.pub_gps.publish(current_fix)

            self.position_covariance_type = current_fix.position_covariance_type
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    gos_node = GPS_Node("gps_node")
    while rclpy.ok:
        gos_node.read_and_publish()
    rclpy.spin(gos_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main(None)
