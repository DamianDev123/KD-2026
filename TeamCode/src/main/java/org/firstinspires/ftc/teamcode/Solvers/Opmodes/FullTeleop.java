package org.firstinspires.ftc.teamcode.Solvers.Opmodes;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import java.util.Objects;

@TeleOp
public class FullTeleop extends CommandOpMode {
    Telemetry telemetry = new TelemetryImplUpstreamSubmission(this);
    Follower follower;
    private final Robot robot = Robot.getInstance();
    TelemetryData telemetryData;
    public GamepadEx driver;
    public GamepadEx operator;
    public String cosl;
    private ElapsedTime elapsedtime;
    private ElapsedTime elapsedtime2;
    private Pose testing = new Pose(97.79939209726444,97.28875379939208,Math.toRadians(45));
    boolean inFull = false;
    private Pose parkPose = new Pose(105.17333333333333,33.22666666666666);
    boolean autoParking = false;
    boolean canDrive = true;
    boolean backup = false;
    boolean intaking = false;
    boolean holdPoint = false;
    boolean holdPointI = false;
    Pose holdPointP = new Pose();
    @Override
    public void initialize() {
        super.reset();
        if(!autoInitialized) {
            follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        }else {
            follower = robot.follower;
        }
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        robot.init(hardwareMap,telemetry,follower);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        telemetryData = robot.telemetryData;
        follower.startTeleOpDrive();

        cosl = ALLIANCE_COLOR;
        if(!autoInitialized){
            if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
                follower.setPose(robot.poses.getStartFromFar().mirror());
                parkPose = parkPose.mirror();
            }else {
                follower.setPose(robot.poses.getStartFromFar());
            }
        }
        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            robot.GoalPose = blueGoalPose;
        }else {
            robot.GoalPose = redGoalPose;
        }
        follower.update();
        Launcher.targetFlywheelVelocity = 0.0;
        elapsedtime = new ElapsedTime();
        elapsedtime2 = new ElapsedTime();
        elapsedtime2.startTime();
        elapsedtime2.reset();
        elapsedtime.reset();

    }

    @Override
    public void run() {
        robot.profiler.start("Full Loop");
                if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3) {
                    robot.intake.intake(Launcher.isFlapOpen == robot.launcher.flapOpen);
                    intaking = true;
                } else {
                    robot.intake.intake(false);
                    intaking = false;
                }
            telemetryData.addData("error", robot.launcher.errorAbs);
            if (driver.getButton(GamepadKeys.Button.A)) {
                inFull = false;

            }
            if (inFull) {
                robot.turret.shouldAim = true;
                robot.launcher.doFlywheel = true;
                robot.launcher.setFlap(true);
            }
            else {
                robot.turret.shouldAim = false;
                robot.launcher.doFlywheel = false;
                robot.launcher.setFlap(false);
            }
            if (driver.getButton(GamepadKeys.Button.B)) {
                inFull = true;
            }
        canDrive = !inFull || !intaking || backup || robot.storage.emptyF || shootingWhileMoving || !autoParking || !robot.inZone();
            holdPoint = canDrive && !autoParking;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                autoPark();
            if (driver.getButton(GamepadKeys.Button.Y)) {
                if (Objects.equals(ALLIANCE_COLOR, "BLUE")) {
                    follower.setPose(robot.poses.getStartFromFar().mirror());
                } else {
                    follower.setPose(robot.poses.getStartFromFar());
                }
                if (Objects.equals(ALLIANCE_COLOR, "BLUE")) {
                    robot.GoalPose = blueGoalPose;
                } else {
                    robot.GoalPose = redGoalPose;
                }
                cosl = ALLIANCE_COLOR;
            }
            if (holdPoint) {
                if(holdPointI)
                    holdPointP = robot.CurrentPose;
                follower.holdPoint(holdPointP);
            }
            Launcher.activeControl = true;
            if(canDrive)
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            robot.updateLoop();
            robot.dashboardTelemetry.addData("Loop times", elapsedtime.milliseconds());
            telemetry.addData("Loop Times", elapsedtime.milliseconds());
            elapsedtime.reset();
        robot.profiler.end("Full Loop");
    }
    public void autoPark() {
        PathChain goToPark = null;
        goToPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                robot.CurrentPose,

                                parkPose
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(90))

                .build();
        follower.followPath(goToPark);
    }
    @Override
    public void end() {
        robot.exportProfiler(robot.file);
    }
}
