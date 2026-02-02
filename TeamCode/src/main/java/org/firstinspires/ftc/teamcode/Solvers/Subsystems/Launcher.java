package org.firstinspires.ftc.teamcode.Solvers.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.MathFunctions;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import static java.lang.Double.NaN;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@Config
public class Launcher extends SubsystemBase {
    public static PIDCoefficients FLYWHEEL_PID_COEFFICIENTS = new PIDCoefficients(0.006, 0, 0); // Coefficients for ticks
    public static BasicFeedforwardParameters FLYWHEEL_FF_COEFFICIENTS = new BasicFeedforwardParameters
            (0.00039, 0,  0.08); // Coefficients for ticks
    private final Robot robot = Robot.getInstance();
    public static Double targetFlywheelVelocity = 0.0;
    public static Double targetFlywheelVelocityIn = 0.0;
    public static Double MeasureVel = 1404.0;
    public static Double MeasureHood = 30.0;

    public static double targetHoodAngle = MIN_HOOD_ANGLE;

    public static double vFeedback = 0;
    TelemetryData telemetryData;
    public static boolean isFlapOpen = false;
    public static boolean activeControl = false;
    public static double tolerances = 100;
    public double errorAbs;
    public double error = 0;
    public boolean inTolerance = false;
    public boolean tolerable = false;

    public boolean doFlywheel = false;
    public boolean doHood = true;
    public boolean doPid = true;
    public static boolean tuning = false;
    public boolean flapOpen = false;

    public static double DISTANCE_OFFSET = -0.267;
    ElapsedTime timer = new ElapsedTime();
    Double distance = 12.0;

    public BasicFeedforwardParameters second =FLYWHEEL_FF_COEFFICIENTS;
    ControlSystem flywheelController = ControlSystem.builder()
            .velPid(FLYWHEEL_PID_COEFFICIENTS)
            .basicFF(second)

            .build();
    public Launcher() {
        telemetryData = robot.telemetryData;
        timer.startTime();
        timer.reset();
    }
    public void setFlap(Boolean onOff) {
        isFlapOpen = onOff;
    }

    public void setHood() {
        double angle2 = Range.clip(targetHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE-3);
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle2 - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }
    public void setFlywheel(double vel) {
        flywheelController.setGoal(new KineticState(0,Math.min(vel, Constants.LAUNCHER_MAX_VELOCITY)));
    }
    public double getFlywheelTarget() {
        return targetFlywheelVelocity;
    }
    public void stopFlywheel(){
        targetFlywheelVelocity = 0.0;
    }
    private void updateFlywheel() {
            second = new BasicFeedforwardParameters(0, FLYWHEEL_FF_COEFFICIENTS.kA, FLYWHEEL_FF_COEFFICIENTS.kS);
            second.kV = FLYWHEEL_FF_COEFFICIENTS.kV * robot.getVoltage();
            if (activeControl) {
                if (doPid)
                    robot.launchMotors.set(flywheelController.calculate(new KineticState(0, robot.launchEncoder.getCorrectedVelocity())));
                else {
                    robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
                }
            } else {
                if (getFlywheelTarget() == 0) {
                    robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
                } else {
                    robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
                }
            }
            setHood();

    }
    @Override
    public void periodic() {

        robot.profiler.start("Launch Loop");
            setFlywheel(targetFlywheelVelocity);
            CalcLauncher();
            error = robot.launchEncoder.getCorrectedVelocity() - targetFlywheelVelocity;
            errorAbs = Math.abs(error);
            robot.flapServo.set(isFlapOpen ? 0.05 : 0.86);
            inTolerance = errorAbs < tolerances;
            robot.dashboardTelemetry.addData("vel", robot.launchEncoder.getCorrectedVelocity());
            double dx = robot.GoalPose.getX() - robot.follower.getPose().getX();
            double dy = robot.GoalPose.getY() - robot.follower.getPose().getY();
            distance = Math.sqrt(dx * dx + dy * dy);
            tolerable = errorAbs < 40;
        robot.dashboardTelemetry.addData("Distance", distance);
        robot.dashboardTelemetry.addData("goal", targetFlywheelVelocity);
            updateFlywheel();
            if(!isFlapOpen)
                timer.reset();
            flapOpen = timer.milliseconds()>100;

        robot.profiler.end("Launch Loop");

    }
    public double getFlywheelTicksFromVelocity(double v){
        return com.pedropathing.math.MathFunctions.clamp(7.5421*v-486.76344, 900,Constants.LAUNCHER_MAX_VELOCITY);
    }
    public void CalcLauncher(){

            double[] s = robot.shootingWhileMoving.calcs;
            targetFlywheelVelocityIn = s[0]/DistanceUnit.mPerInch;
            if (Double.isNaN(targetFlywheelVelocityIn))
                targetFlywheelVelocityIn = 0.0;
            double vel = getFlywheelTicksFromVelocity(targetFlywheelVelocityIn);
            if (Double.isNaN(s[1]))
                s[1] = 19;
            if (doHood) {
                if (tuning)
                    targetHoodAngle = MeasureHood - error * vFeedback + 1;
                else
                    targetHoodAngle = s[1];

            }
            if (doFlywheel) {
                if (tuning)
                    targetFlywheelVelocity = MeasureVel;
                else {
                    targetFlywheelVelocity = vel;
                    if (robot.turret.getRunningAuto())
                        targetFlywheelVelocity = vel;
                }
                doPid = true;
            } else {
                doPid = false;
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }

    }



}
