import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import Int32
from datetime import datetime, timedelta
import threading
import time

# OpenWeatherMap API Settings
API_KEY = "9ef62c6aff8bd0157252f1901262f3de"  # Replace with your OpenWeatherMap API key
LAT = "42.98"   # Exeter, NH Latitude
LON = "-70.95"  # Exeter, NH Longitude
WEATHER_API_URL = f"https://api.openweathermap.org/data/2.5/forecast?lat={LAT}&lon={LON}&appid={API_KEY}&units=imperial"

class WeatherPublisher(Node):
    def __init__(self):
        super().__init__("weather_publisher")
        self.publisher_ = self.create_publisher(Int32, "good_weather", 10)
        self.last_weather_status = None  # Store last fetched weather condition

        # Fetch first weather data immediately
        self.fetch_and_publish_weather()

        # Start background thread for scheduled updates
        thread = threading.Thread(target=self.schedule_next_update, daemon=True)
        thread.start()

        # Start continuous publishing of last known value
        self.timer_ = self.create_timer(1, self.publish_last_value)  # Publish every 1 second

    def fetch_weather_data(self):
        """Fetch weather data from OpenWeatherMap API"""
        try:
            response = requests.get(WEATHER_API_URL)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f"❌ Error fetching weather data: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Exception fetching weather data: {e}")
        return None

    def is_good_weather(self, weather_data):
        """Check if the weather conditions meet 'good weather' criteria"""
        try:
            now = datetime.utcnow()
            total_rain = 0.0
            good_temp_count = 0

            # Ensure we get forecasts within the next 3 hours
            valid_forecasts = []
            for forecast in weather_data["list"]:
                forecast_time = datetime.strptime(forecast["dt_txt"], "%Y-%m-%d %H:%M:%S")
                if now <= forecast_time <= now + timedelta(hours=3):
                    valid_forecasts.append(forecast)

                if len(valid_forecasts) >= 3:
                    break  # Stop when we have enough forecasts

            # Check if all selected forecasts meet the good weather criteria
            for forecast in valid_forecasts:
                temp_f = forecast["main"]["temp"]
                rain = forecast.get("rain", {}).get("3h", 0.0)  # Rain in mm

                total_rain += rain
                if temp_f > 60:
                    good_temp_count += 1

            # Good weather: Temp > 60°F for all 3 periods & No rain
            return good_temp_count == len(valid_forecasts) and total_rain == 0.0
        except Exception as e:
            self.get_logger().error(f"❌ Error processing weather data: {e}")
            return False

    def fetch_and_publish_weather(self):
        """Fetch new weather data and update the last stored value"""
        weather_data = self.fetch_weather_data()
        if not weather_data:
            self.get_logger().warn("⚠️ No weather data retrieved. Retaining last known value.")
            return

        new_status = 1 if self.is_good_weather(weather_data) else 0  # Yes → 1, No → 0
        if new_status != self.last_weather_status:
            self.last_weather_status = new_status
            self.get_logger().info(f"🌦️ Updated: Good weather is now ({'Yes' if new_status == 1 else 'No'})")

    def publish_last_value(self):
        """Continuously publish the last known weather status"""
        if self.last_weather_status is not None:
            msg = Int32()
            msg.data = self.last_weather_status
            self.publisher_.publish(msg)
            self.get_logger().info(f"📡 Publishing: {self.last_weather_status}")

    def schedule_next_update(self):
        """Schedules the next weather update at the start of the next hour"""
        while rclpy.ok():
            now = datetime.now()
            next_hour = (now + timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)
            delay = (next_hour - now).total_seconds()

            self.get_logger().info(f"⏳ Next weather update scheduled at {next_hour.strftime('%H:%M:%S')} (in {int(delay)} seconds)")
            time.sleep(delay)  # Correctly sleep until the next hour
            
            # Fetch new data and update stored value
            self.fetch_and_publish_weather()

def main(args=None):
    rclpy.init(args=args)
    node = WeatherPublisher()
    rclpy.spin(node)  # Keeps ROS 2 running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
