# Studica 1.1.8 compatibility implementation

The public class names, constructors and methods follow the user-supplied
Titan-cpp-1.1.8 headers and Titan-java-1.1.8 sources (Studica Limited, 2020).
The implementation is new code for WPILib 2026; no original TitanDriver binary,
JNI implementation or proprietary VMX SDK is redistributed.

Titan CAN message identifiers, payload layouts, little-endian encoder counts,
RPM/100 telemetry and the Cobra ADS1015 configuration were derived from
Studica Robotics' Apache-2.0 ROS2 drivers at commit
`f97edecac9175107b2b6cff841264fcaed912fb3`:
https://github.com/Studica-Robotics/ROS2/tree/f97edecac9175107b2b6cff841264fcaed912fb3/drivers

The upstream license is included as `Studica-ROS2-LICENSE.txt`.

The original Studica 360-degree LiDAR uses the YDLIDAR non-intensity triangle
packet format. Its packet/checksum and angle-correction implementation follows
the published protocol description (not SDK source code):
https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md
The Studica sensor documentation links to the YDLIDAR ROS/ROS2 drivers:
https://docs.dev.studica.com/en/latest/docs/Sensors/lidar.html

New files are distributed under the repository's WPILib BSD license, except
for the upstream license text itself. Refer to the repository LICENSE.md.
