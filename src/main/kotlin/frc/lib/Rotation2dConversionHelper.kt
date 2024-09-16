package frc.lib

import edu.wpi.first.math.geometry.Rotation2d

fun Double.rotation2dFromDeg() = Rotation2d.fromDegrees(this)
fun Float.rotation2dFromDeg() = Rotation2d.fromDegrees(this.toDouble())
fun Int.rotation2dFromDeg() = Rotation2d.fromDegrees(this.toDouble())


fun Double.rotation2dFromRad() = Rotation2d.fromRadians(this)
fun Float.rotation2dFromRad() = Rotation2d.fromRadians(this.toDouble())
fun Int.rotation2dFromRad() = Rotation2d.fromRadians(this.toDouble())


fun Double.rotation2dFromRot() = Rotation2d.fromRotations(this)
fun Float.rotation2dFromRot() = Rotation2d.fromRotations(this.toDouble())
fun Int.rotation2dFromRot() = Rotation2d.fromRotations(this.toDouble())

