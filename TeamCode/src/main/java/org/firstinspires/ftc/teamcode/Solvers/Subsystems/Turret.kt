package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.TelemetryData
import org.firstinspires.ftc.teamcode.Globals.Constants.redTurretGoalPose
import org.firstinspires.ftc.teamcode.Globals.Constants.redX
import org.firstinspires.ftc.teamcode.Globals.Constants.redY
import org.firstinspires.ftc.teamcode.Globals.Robot
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.testOp
import kotlin.math.atan2


@Config
class Turret : Subsystem() {
    private val robot: Robot = Robot.getInstance()
    var telemetryData: TelemetryData = robot.telemetryData
    var runningAuto = false;
    var follower: Follower = robot.follower
    var turretServo2: ServoEx = robot.turretServo2
    var turretServo1: ServoEx = robot.turretServo1
    private val ppr = 4000
    private val gearRatio = 90.0 / 35.0
    private val ticksPerDeg = (ppr * gearRatio) / 360
    var elapsedTime: ElapsedTime = ElapsedTime();
    @JvmField
    var tolerable = false;
    @JvmField
    var shouldAim = false;
    @JvmField
    var toggle = true;
    init {
        elapsedTime.startTime()
        elapsedTime.reset()
    }

    fun update() {


        robot.profiler.start("Turret Loop")
            var target = -CalculateGoal();
            // 1. Find the "Range" (how many servo units represent 90 degrees)s
            var servoPosition = testOp.forward;
            if (shouldAim)
                servoPosition =
                    testOp.forward + (target / 90.0) * (testOp.right - testOp.forward)
            turretServo1.set(servoPosition)
            turretServo2.set(servoPosition)
            val error = Math.abs(robot.turretEncoder.position / ticksPerDeg + target)
            telemetryData.addData("error", error);
            val ntolerable = error > 6
            if (ntolerable)
                elapsedTime.reset()
            tolerable = elapsedTime.milliseconds() > 300;

        robot.profiler.end("Turret Loop")


    }

    override fun periodic() {
        update()
    }
    private fun CalculateGoal(): Double {

        var mu = atan2( redY - ShootingWhileMoving.predictedPose.y, redX- ShootingWhileMoving.predictedPose.x)
        var deltaHeading = Math.toDegrees(mu) - Math.toDegrees(robot.pose.heading)
        return deltaHeading.coerceIn(-90.0,90.0);
    }


}
