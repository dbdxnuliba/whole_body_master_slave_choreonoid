# WholeBodyPositionController

## InPort
- `qRef` (RTC::TimedDoubleSeq) [REQUIRED]
- `basePosRef` (RTC::TimedPoint3D) [REQUIRED]
- `baseRpyRef` (RTC::TimedOrientation3D) [REQUIRED]
- `primitiveCommandRef` (primitive_motion_level_msgs::TimedPrimitiveStateSeq) [REQUIRED]

## OutPort
- `qCom` (RTC::TimedDoubleSeq)
- `basePosCom` (RTC::TimedPoint3D)
- `baseRpyCom` (RTC::TimedOrientation3D)

## ServiceServer
- `service0` (whole_body_position_controller::WholeBodyPositionControllerService)