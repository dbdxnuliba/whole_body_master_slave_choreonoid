# PrimitiveMotionLevelController

## InPort
- `qRef` (RTC::TimedDoubleSeq) [REQUIRED]
- `basePosRef` (RTC::TimedPoint3D) [REQUIRED]
- `baseRpyRef` (RTC::TimedOrientation3D) [REQUIRED]
- `primitiveCommandRef` (WholeBodyMasterSlaveChoreonoidIdl::TimedPrimitiveStateSeq) [REQUIRED]
- `qAct` (RTC::TimedDoubleSeq) [REQUIRED]
- `imuAct` (RTC::TimedOrientation3D) [REQUIRED]

## OutPort
- `qCom` (RTC::TimedDoubleSeq)
- `basePosCom` (RTC::TimedPoint3D)
- `baseRpyCom` (RTC::TimedOrientation3D)

## ServiceServer
- `service0` (WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService)