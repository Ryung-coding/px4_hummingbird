# Orin DDS gap check

Run this on Orin. It only subscribes and logs; it never publishes commands or starts the Agent. No Opti node is needed.

Remove propellers and secure the airframe before any powered servo or Agent restart test. Keep the usual PX4 firmware and one Agent running:

```bash
sudo env ROS_DOMAIN_ID=13 MicroXRCEAgent serial --dev /dev/dds -b 921600 -v 2 2>&1 | tee ~/agent_test.log
```

In another Orin terminal, run the servo node if servo behavior is being tested. Leave it in the foreground so any exit or Dynamixel error is visible:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/px4_hummingbird/install/setup.bash
export ROS_DOMAIN_ID=13
ros2 run px4_hummingbird_cmd px4_servo_to_dynamixel 2>&1 | tee ~/servo_test.log
```

Run the monitor in a third terminal:

```bash
cd ~/Desktop/px4_hummingbird
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
python3 debug/dds_watch.py
```

Disarmed PX4 may not publish `actuator_servos`; `servo=0Hz` alone is not a DDS fault. When servo output is expected, add `--expect-servos`. The default gap threshold is 0.2 s, matching the servo bridge's DDS receive timeout. Stop with Ctrl-C. Each run writes `debug/logs/dds_<UTC>.jsonl`; this directory is ignored by Git.

The JSONL contains every received sample, receipt time, PX4 timestamp, servo control values, detected gaps/silences, publisher counts, and Agent/servo process PID changes. `agent=[]` or `servo=[]` means the corresponding executable was not found on the same Orin. The monitor observes ROS topics locally: seeing an `/fmu/in/...` sample does **not** prove PX4 accepted it. Compare with `uxrce_dds_client status` on PX4 for `connected` and TX/RX payload rates. Rates in the terminal are 1-second windows; the JSONL gap events are more useful for intermittent stalls. A topic may appear in the ROS graph without delivering samples.

To test reconnection without Opti, keep PX4 disarmed, stop only the Agent for a few seconds, restart it, and check that `/fmu/out/vehicle_attitude` resumes. A later servo test requires actual PX4 servo output with propellers removed. If the Agent returns but the servo executable has exited, inspect `~/servo_test.log` for `Dynamixel output failed` or `Dynamixel output timeout`. Do not infer a servo failure from silence while disarmed.
