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
import frc.lib.IntakeSubsystemIO
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
class IntakeSubsystemIOSim(val index: Int) : IntakeSubsystemIO {
    // these motors are probably wrong idk
    private val armSim = DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025)
    private val intakeSim = DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004)

    private val turnAbsoluteInitPosition = Rotation2d(Math.random() * 2.0 * Math.PI)
    private var armAppliedVolts = 0.0
    private var intakeAppliedVolts = 0.0
    override val turnPIDController: PIDController = PIDController(0.0,0.0,0.0)

    override fun updateInputs(inputs: IntakeSubsystemIO.IntakeIOInputs) {
        armSim.update(LOOP_PERIOD_SECS)
        intakeSim.update(LOOP_PERIOD_SECS)

        // This probably doesn't work
        inputs.armPosition = Rotation2d(intakeSim.angularPositionRad)
        inputs.armVelocityRadPerSec = armSim.angularVelocityRadPerSec
        inputs.armAppliedVolts = armAppliedVolts
        inputs.armCurrentAmps = doubleArrayOf(abs(armSim.currentDrawAmps))

        inputs.intakePosition = Rotation2d(intakeSim.angularPositionRad)
        inputs.intakeVelocityRadPerSec = intakeSim.angularVelocityRadPerSec
        inputs.intakeAppliedVolts = intakeAppliedVolts
        inputs.intakeCurrentAmps = doubleArrayOf(abs(intakeSim.currentDrawAmps))
    }

    override fun setArmVoltage(volts: Double) {
        armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        armSim.setInputVoltage(armAppliedVolts)
    }

    override fun setIntakeVoltage(volts: Double) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        intakeSim.setInputVoltage(intakeAppliedVolts)
    }

    companion object {
        private const val LOOP_PERIOD_SECS = 0.02
    }
}