cmake_minimum_required(VERSION 3.8)
project(drop_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeltMotion.msg"
  "msg/Heartbeat.msg"
  "msg/InterceptPlan.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/RobotStatus.msg"
  "msg/StepRecord.msg"
  "msg/TaskStatus.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "srv/ComputeIntercept.srv"
  "srv/DetectObjects.srv"
  "srv/ResetTracking.srv"
  "srv/SelectTarget.srv"
  "srv/Stop.srv"
  "srv/VerifyDrop.srv"
  "action/DropTask.action"
  "action/DynamicPlace.action"
  "action/MoveToPose.action"
  "action/PickStone.action"
  "action/PlaceStone.action"
  "action/SafeRetreat.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
