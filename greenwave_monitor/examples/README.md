# Greenwave Monitor Examples

This folder contains example launch files and configurations for using the greenwave_monitor.

## example.launch.py

This launch file demonstrates how to integrate the greenwave monitor node into your system:

- Launches 3 minimal publisher nodes publishing with different message types (imu, image, string)
- Launches the greenwave_monitor to monitor all three topics

### Usage

```bash
ros2 launch greenwave_monitor example.launch.py
```

### Integration into Your Launch Files

To add monitoring to your existing launch files, include the monitor node like this:

```python
Node(
    package='greenwave_monitor',
    executable='greenwave_monitor',
    name='greenwave_monitor',
    output='screen',  # or 'log' if you want to add monitoring without terminal output
    parameters=[
        {'topics': ['/your_topic_1', '/your_topic_2']}  # List your topics to monitor
    ],
),
```

To see the output with the r2s_gw dashboard, run `ros2 run r2s_gw r2s_gw` in a separate terminal.