package org.ghrobotics.lib.mathematics.twodim.trajectory

import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.controller.LTVUnicycleController
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
            val linear = wheelSpeeds.vxMetersPerSecond.coerceIn(-4.0, 4.0)

            val twist = Twist2d(
                linear * deltaTime.inSeconds() * 0.8,
                0.0,
                (wheelSpeeds.omegaRadiansPerSecond * 1.3 * deltaTime.inSeconds())
            )

            val newPose = robotLocation.exp(twist)

            robotLocation = newPose
        }
    }

    @Test
    fun testLTVUnicycleController() {

        val trajectory = FalconTrajectoryGenerator.generateTrajectory(
            listOf(Pose2d(0.feet, 0.feet, 0.degrees), Pose2d(10.feet, 10.feet, 0.degrees), Pose2d(10.feet, 10.feet, 180.degrees)),
            FalconTrajectoryConfig(10.feet.velocity, 10.feet.acceleration)
        )

        val controller = LTVUnicycleController(34.86245213903779, 38.91335252924404, 35.7834141375445, 8.470494136209277) // RamseteController(2.0, 0.7) //defaultLTVUnicycleController
        val drive = SimDiffDrive()

        drive.robotLocation = Pose2d(0.feet, 2.feet, 45.degrees)

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

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)

    }

}