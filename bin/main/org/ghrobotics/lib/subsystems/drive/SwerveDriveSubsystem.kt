package org.ghrobotics.lib.subsystems.drive

import org.ghrobotics.lib.components.DriveComponent
import org.ghrobotics.lib.components.EmergencyHandleable
import org.ghrobotics.lib.components.SwerveDriveComponent
import org.ghrobotics.lib.components.SwerveModuleComponent
import org.ghrobotics.lib.localization.SwerveDriveLocalization
import org.ghrobotics.lib.physics.SwerveDriveKinematics

class SwerveDriveSubsystem(
        override val modules: List<SwerveModuleComponent>,
        override val kinematics: SwerveDriveKinematics,
        override val localization: SwerveDriveLocalization,
        drivetrainHeightFromGround: Double
) : SwerveDriveComponent(drivetrainHeightFromGround), EmergencyHandleable {

    override fun activateEmergency(severity: EmergencyHandleable.Severity) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun recoverFromEmergency() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }



}