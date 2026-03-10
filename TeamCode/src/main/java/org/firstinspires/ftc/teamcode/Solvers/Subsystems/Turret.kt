package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.Globals.Constants
import org.firstinspires.ftc.teamcode.Globals.Robot
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem
import kotlin.math.atan2

@Configurable
class Turret : Subsystem() {
    private val robot: Robot = Robot.getInstance()
    @JvmField
    var inAuto = true;
    var runningAuto = false;
    var override = false;
    @JvmField
    var overrided = 0.0;
    @JvmField
    var forcedPos = Pose(0.0,0.0)

    companion object {
        @JvmField var tuningOffset = false

        @JvmField var targetAngle = 90.0;

        @JvmField var offsetB = 0.0;
        @JvmField var offsetR = 0.0;
        @JvmField var offsetRBack = -0.0;
        @JvmField var headingOffset = 0.0;
        @JvmField var xOffset = 0.0
        @JvmField var yOffset = 0.0
    }
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
    fun headingScalar(heading: Double) : Double{
        val x = ((heading + 180) % 360 + 360) % 360 - 180;
        robot.telemetry.addData("heading", heading)
        val y = -0.0000667247*x*x-0.078323*x-0.984867
        return heading+y;
    }
    fun xScalar(x: Double) : Double{

        return x+xOffset;
    }
    fun yScalar(y: Double) : Double{
        robot.telemetry.addData("y", y)
        return y-yOffset;
        ;
    }
    fun update() {


        robot.profiler.start("Turret Loop")
        var target = 90.0;
        target = normalizeDegrees(-CalculateGoal()+90);
        if(tuningOffset)
            target = 90.0;
            // 1. Find the "Range" (how many servo units represent 90 degrees)s
        target = Math.toDegrees(MathFunctions.normalizeAngle(Math.toRadians(target)));
        if(target>180.0){
            target = 0.0;
        }

        var of = offsetR
        if(robot.intake.runningAuto )
            of -= 10;

        if(Constants.ALLIANCE_COLOR == "BLUE"){
            of = offsetB
            if(robot.intake.runningAuto )
                of += 10;

        }
        var servoPosition = (((target+of)*(90.0/48.0))/(355))

        turretServo1.set(servoPosition)
        turretServo2.set(servoPosition)

        val error = Math.abs(robot.turretEncoder.position / ticksPerDeg + target)
        val ntolerable = error > 4
        if (ntolerable)
            elapsedTime.reset()
        tolerable = elapsedTime.milliseconds() > 300;
        robot.telemetry.update()

        robot.profiler.end("Turret Loop")

        robot.telemetry.addData("heading:", robot.follower.heading - 45)
    }
    override fun periodic() {
        update()
    }
    fun overrideTurret(p: Pose){
        forcedPos = p;
    }
    fun CalculateGoal(): Double {
        var currentP = robot.limelight.truePose;
        val mu = atan2(yScalar(robot.PredictedGoalPose.y - currentP.y), xScalar(robot.PredictedGoalPose.x - currentP.x))
        val deltaHeading = mu-robot.pose.heading
        return headingScalar(Math.toDegrees(deltaHeading))
    }
    fun normalizeDegrees(angle: Double): Double {
        var angle = angle % 360;
        if(angle>180){
            angle -=360
        }else if(angle <= -180){
            angle+-360;
        }
        return  angle;
    }
}
