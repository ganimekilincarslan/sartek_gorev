import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# pyrefly: ignore[missing-import]
from sartek_msgs.msg import MissionCommand, TelemetryStatus


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__('waypoint_mission_node')

        # Sabit kanatlı İHA için uçuş parametreleri
        self.acceptance_radius_m = 25.0   # Hedefe yatayda 25m yaklaşınca geç
        self.min_takeoff_alt_m = 15.0     # QGC kalkışının tamamlandığını anlama irtifası
        self.cruise_speed_mps = 18.0      # Seyir hızı (m/s)
        self.rtl_wait_sec = 60.0          # RTL'de ne kadar süre (saniye) kalsın?

        # Görev durumu ve waypoint kuyruğu
        self.waypoints = []               # [(x, y, z), ...]
        self.current_wp_index = 0
        self.state = 'WAIT_TAKEOFF'       # WAIT_TAKEOFF -> OFFBOARD_NAV -> RTL_WAIT -> LAND_DONE
        self.telemetry = None
        self.offboard_requested = False
        self.rtl_start_time = None

        # Yayıncılar (Publishers)
        self.mission_pub = self.create_publisher(MissionCommand, '/gorev/mission_command', 10)
        self.vehicle_cmd_pub = self.create_publisher(String, '/gorev/vehicle_command', 10)

        # Dinleyiciler (Subscribers)
        self.create_subscription(TelemetryStatus, '/gorev/telemetry_status', self.telemetry_cb, 10)
        self.create_subscription(String, '/yer_istasyonu/waypoints', self.waypoint_cb, 10)

        # Kontrol döngüsü (5 Hz)
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('Görev düğümü hazır. QGC kalkışı ve yer istasyonundan waypoint bekleniyor...')

    def telemetry_cb(self, msg: TelemetryStatus):
        self.telemetry = msg

    def waypoint_cb(self, msg: String):
        """Toplu (liste) veya parça parça (tek sözlük) gelen JSON waypoint'leri kuyruğa ekler."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                data = [data]

            eklenen = 0
            for wp in data:
                self.waypoints.append((float(wp['x']), float(wp['y']), float(wp['z'])))
                eklenen += 1

            self.get_logger().info(f'{eklenen} waypoint eklendi. Toplam rota: {len(self.waypoints)} nokta.')
        except Exception as e:
            self.get_logger().error(f'Waypoint JSON hatası: {e}')

    def send_vehicle_command(self, cmd_text: str):
        msg = String()
        msg.data = cmd_text
        self.vehicle_cmd_pub.publish(msg)
        self.get_logger().info(f'Araç komutu gönderildi -> {cmd_text}')

    def send_goto(self, x: float, y: float, z: float, state_label: str):
        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'enu'
        cmd.command_type = 'GOTO'
        cmd.target_x = float(x)
        cmd.target_y = float(y)
        cmd.target_z = float(z)
        cmd.speed = float(self.cruise_speed_mps)
        cmd.state_name = state_label
        cmd.is_dynamic_target = False
        cmd.commander_id = 'waypoint_mission'
        self.mission_pub.publish(cmd)

    def send_land(self):
        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'enu'
        cmd.command_type = 'LAND'
        cmd.target_x = 0.0
        cmd.target_y = 0.0
        cmd.target_z = 0.0
        cmd.speed = float(self.cruise_speed_mps)
        cmd.state_name = 'LANDING'
        cmd.is_dynamic_target = False
        cmd.commander_id = 'waypoint_mission'
        self.mission_pub.publish(cmd)
        self.get_logger().info('RTL bekleme süresi doldu -> LAND (İniş) komutu gönderildi!')

    def control_loop(self):
        if self.telemetry is None or not self.telemetry.is_connected:
            return

        # 1. AŞAMA: QGC kalkışı ve en az 1 waypoint gelmesini bekle
        if self.state == 'WAIT_TAKEOFF':
            if self.telemetry.altitude >= self.min_takeoff_alt_m and len(self.waypoints) > 0:
                tx, ty, tz = self.waypoints[0]
                self.send_goto(tx, ty, tz, 'PREPARE_OFFBOARD')
                if not self.offboard_requested:
                    self.send_vehicle_command('OFFBOARD')
                    self.offboard_requested = True
                    self.state = 'OFFBOARD_NAV'
                    self.get_logger().info(
                        f'Kalkış algılandı ({self.telemetry.altitude:.1f} m). '
                        f'Offboard moduna geçildi -> Hedef WP 1: ({tx}, {ty}, {tz})'
                    )

        # 2. AŞAMA: Waypoint'leri sırayla takip et
        elif self.state == 'OFFBOARD_NAV':
            if self.current_wp_index >= len(self.waypoints):
                self.send_vehicle_command('RTL')
                self.rtl_start_time = time.time()
                self.state = 'RTL_WAIT'
                self.get_logger().info(
                    f'Tüm waypoint noktaları tamamlandı! RTL başlatıldı. '
                    f'{self.rtl_wait_sec:.0f} saniye sonra inişe geçilecek...'
                )
                return

            tx, ty, tz = self.waypoints[self.current_wp_index]
            self.send_goto(tx, ty, tz, f'GOTO_WP_{self.current_wp_index + 1}')

            dx = tx - self.telemetry.pos_x
            dy = ty - self.telemetry.pos_y
            horiz_dist = math.hypot(dx, dy)
            alt_err = abs(tz - self.telemetry.pos_z)

            if horiz_dist <= self.acceptance_radius_m:
                self.get_logger().info(
                    f'WP {self.current_wp_index + 1} tamamlandı! '
                    f'(Yatay sapma: {horiz_dist:.1f} m, İrtifa farkı: {alt_err:.1f} m)'
                )
                self.current_wp_index += 1
                if self.current_wp_index < len(self.waypoints):
                    nx, ny, nz = self.waypoints[self.current_wp_index]
                    self.get_logger().info(f'Yeni hedef WP {self.current_wp_index + 1}: ({nx}, {ny}, {nz})')

        # 3. AŞAMA: RTL'de belirlenen süre kadar bekle, sonra LAND gönder
        elif self.state == 'RTL_WAIT':
            elapsed = time.time() - self.rtl_start_time
            if elapsed >= self.rtl_wait_sec:
                self.send_land()
                self.state = 'LAND_DONE'

        # 4. AŞAMA: İniş komutu gönderildi
        elif self.state == 'LAND_DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
