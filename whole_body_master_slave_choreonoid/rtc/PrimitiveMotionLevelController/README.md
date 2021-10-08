# PrimitiveMotionLevelController

## InPort
- `qRef` (RTC::TimedDoubleSeq) [REQUIRED]
- `basePosRef` (RTC::TimedPoint3D) [REQUIRED]
- `baseRpyRef` (RTC::TimedOrientation3D) [REQUIRED]
- `primitiveCommandRef` (whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq) [REQUIRED]

## OutPort
- `qCom` (RTC::TimedDoubleSeq)
- `basePosCom` (RTC::TimedPoint3D)
- `baseRpyCom` (RTC::TimedOrientation3D)

## ServiceServer
- `service0` (whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService)