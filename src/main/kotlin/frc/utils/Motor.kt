package frc.utils

import com.revrobotics.*

class Motor(deviceId: Int) : CANSparkMax(deviceId, MotorType.kBrushless)