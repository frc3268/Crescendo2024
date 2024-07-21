// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.lib.SwerveModuleIO
import frc.lib.SwerveModuleIO.ModuleIOInputs
import kotlin.math.abs

/**
 * Physics sim implementation of module IO.
 *
 *
 * Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
class SwerveModuleIOSim(val index: Int) : SwerveModuleIO {
    private val driveSim = DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025)
    private val turnSim = DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004)

    private val turnAbsoluteInitPosition = Rotation2d(Math.random() * 2.0 * Math.PI)
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0
    override val turnPIDController: PIDController = PIDController(0.0,0.0,0.0)

    override fun updateInputs(inputs: ModuleIOInputs) {
        driveSim.update(LOOP_PERIOD_SECS)
        turnSim.update(LOOP_PERIOD_SECS)

        //FIX
        inputs.driveVelocityMetersPerSec = driveSim.angularPositionRad
        inputs.driveVelocityMetersPerSec = driveSim.angularVelocityRadPerSec
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = doubleArrayOf(abs(driveSim.currentDrawAmps))

        inputs.turnAbsolutePosition =
            Rotation2d(turnSim.angularPositionRad).plus(turnAbsoluteInitPosition)
        inputs.turnPosition = Rotation2d(turnSim.angularPositionRad)
        inputs.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrentAmps = doubleArrayOf(abs(turnSim.currentDrawAmps))
    }

    override fun setDriveVoltage(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveSim.setInputVoltage(driveAppliedVolts)
    }

    override fun setTurnVoltage(volts: Double) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        turnSim.setInputVoltage(turnAppliedVolts)
    }

    companion object {
        private const val LOOP_PERIOD_SECS = 0.02
    }
}