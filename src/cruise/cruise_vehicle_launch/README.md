# cruise_vehicle_launch

Vehicle launch package for cruise control scenarios, based on [sample_vehicle_launch](https://github.com/autowarefoundation/autoware_launch).

## Packages

- **cruise_vehicle_description**: Vehicle model description (URDF/Xacro), parameters for vehicle info, simulator model, and mirror configuration.
- **cruise_vehicle_launch**: Launch files for vehicle interface.

## Usage

Build and source the workspace, then launch with:

```bash
ros2 launch cruise_vehicle_launch vehicle_interface.launch.xml
```

## Vehicle Configuration

The default vehicle is configured with Lexus parameters (wheel base 2.79m, wheel tread 1.64m, etc.). Modify `cruise_vehicle_description/config/vehicle_info.param.yaml` for different vehicle specifications.
