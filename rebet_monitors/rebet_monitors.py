# Copyright 2025 KAS-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque
from nav_msgs.msg import Odometry
from rebet_msgs.srv import DetectObject
import math
from ros_monitors_functions.laser_scan_functions import get_nearest_laser_scan_ratio
from ros_monitors_functions.utils import register_monitor

@register_monitor("is_too_close")
def is_too_close(_, laser_scan_msg):
    ratio = get_nearest_laser_scan_ratio(_, laser_scan_msg)
    return ratio < 0.1


@register_monitor("MovementPowerDraw")
class MovementPowerDraw:
    """
    Callable monitor that maintains a rolling mean linear speed over a time window.

    Example:
    -------
        move_power = MovementPowerDraw()
        node.create_subscription(Odometry, 'odom', move_power, qos_profile)

    """

    def __init__(self, threshold: float = 0.001):
        self.window_seconds = 1.0
        self.threshold = threshold
        self._samples = deque()  # holds (timestamp_sec, speed)
        self._running_sum = 0.0

    def __call__(self, _, odom_msg: Odometry) -> float:

        # Convert message timestamp to float seconds
        current_ts = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # Compute instantaneous speed with the same small-value clamp as calc_linear_velocity
        vx = 0.0 if abs(odom_msg.twist.twist.linear.x) < self.threshold \
            else odom_msg.twist.twist.linear.x
        vy = 0.0 if abs(odom_msg.twist.twist.linear.y) < self.threshold \
            else odom_msg.twist.twist.linear.y
        speed = math.hypot(abs(vx), abs(vy))

        # Append new sample and trim old ones outside the window
        self._samples.append((current_ts, speed))
        self._running_sum += speed

        window_start = current_ts - self.window_seconds  # one second before right now
        while self._samples and self._samples[0][0] < window_start:  # remove old samples
            _, old_speed = self._samples.popleft()
            self._running_sum -= old_speed

        if not self._samples:
            return 0.0

        mean_linear_velocity = self._running_sum / len(self._samples)

        return 6.25 * pow(mean_linear_velocity, 2) + 9.79 * mean_linear_velocity + 3.66


@register_monitor("PowerBudget")
class PowerBudget:
    """
    Callable monitor that maintains a rolling mean linear speed over a time window.

    Example:
    -------
        pow_bud = PowerBudget(num_obstacles=4, pics_per_obs=5, min_pics=1,
        freedom_to_det_extra=2.5, avg_power_detection=59.27, pic_increment=2):

        //TODO
    """

    def __init__(self, num_obstacles=4, pics_per_obs=5, min_pics=1, freedom_to_det_extra=2.5,
                 avg_power_detection=59.27, pic_increment=2):
        self.avg_power_detection = avg_power_detection
        self.power_budget = (num_obstacles * pics_per_obs) * \
            (freedom_to_det_extra * (pic_increment + min_pics)) * \
            avg_power_detection

    def __call__(self, _, det_obj: DetectObject.Response) -> float:
        self.power_budget = self.power_budget - \
            (len(det_obj.objects_id) * self.avg_power_detection)
        return self.power_budget
