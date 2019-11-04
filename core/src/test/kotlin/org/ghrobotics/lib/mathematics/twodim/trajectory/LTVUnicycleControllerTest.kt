package org.ghrobotics.lib.mathematics.twodim.trajectory

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.RamseteCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.controller.defaultLTVUnicycleController
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat

@Suppress("UNREACHABLE_CODE")
class LTVUnicycleControllerTest {

    class SimDiffDrive {
        var robotLocation = Pose2d(0.meters, 0.meters, 0.radians.toRotation2d())

        fun update(deltaTime: SIUnit<Second>, wheelSpeeds: ChassisSpeeds) {
            val twist = Twist2d(
                wheelSpeeds.vxMetersPerSecond * deltaTime.inSeconds(),
                0.0,
                (wheelSpeeds.omegaRadiansPerSecond * deltaTime.inSeconds())
            )

            val newPose = robotLocation.exp(twist)

            robotLocation = newPose
        }
    }

    @Test
    fun testLTVUnicycleController() {

        val trajectory = FalconTrajectoryGenerator.generateTrajectory(
            listOf(Pose2d(0.feet, 0.feet, 0.degrees), Pose2d(10.feet, 10.feet, 0.degrees)),
            FalconTrajectoryConfig(10.feet.velocity, 10.feet.acceleration)
        )

        val controller = defaultLTVUnicycleController // RamseteController(2.0, 0.7) //defaultLTVUnicycleController
        val drive = SimDiffDrive()

        drive.robotLocation = Pose2d(0.feet, 2.feet, -50.degrees)

        var currentTime = 0.second
        val deltaTime = 20.milli.seconds

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val refXList = arrayListOf<Double>()
        val refYList = arrayListOf<Double>()

        while(currentTime.inSeconds() < trajectory.totalTimeSeconds) {

            val nextTime = currentTime + deltaTime
            val state = trajectory.sample(nextTime.inSeconds())

            val output = controller.calculate(drive.robotLocation, state)

            drive.update(deltaTime, output)

            val newX = drive.robotLocation.translation.x
            val newY = drive.robotLocation.translation.y

            xList += newX.meters.inFeet()
            yList += newY.meters.inFeet()

            val wantedX = state.poseMeters.translation.x
            val wantedY = state.poseMeters.translation.y

            refXList += wantedX.meters.inFeet()
            refYList += wantedY.meters.inFeet()

            currentTime = nextTime

        }

        val fm = DecimalFormat("#.###").format(trajectory.totalTimeSeconds)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
            .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = -1.0
        chart.styler.xAxisMax = refXList.max()
        chart.styler.yAxisMin = -1.0
        chart.styler.yAxisMax = refXList.max()

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Trajectory", refXList.toDoubleArray(), refYList.toDoubleArray())
        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

//        val terror =
//            TrajectoryGeneratorTest.trajectory.lastState.state.pose.translation - drive.robotLocation.translation
//        val rerror = TrajectoryGeneratorTest.trajectory.lastState.state.pose.rotation - drive.robotLocation.rotation

//        System.out.printf("%n[Test] X Error: %3.3f, Y Error: %3.3f%n", terror.x.feet, terror.y.feet)

//        assert(terror.norm.value.also {
//            println("[Test] Norm of Translational Error: $it")
//        } < 0.50)
//        assert(rerror.degree.also {
//            println("[Test] Rotational Error: $it degrees")
//        } < 5.0)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)

    }

}