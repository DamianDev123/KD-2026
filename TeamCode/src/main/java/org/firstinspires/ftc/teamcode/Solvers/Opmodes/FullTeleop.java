package org.firstinspires.ftc.teamcode.Solvers.Opmodes;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import android.graphics.Color;

import java.util.Objects;

@TeleOp
public class FullTeleop extends CommandOpMode {
    Telemetry telemetry = new TelemetryImplUpstreamSubmission(this);
    Follower follower;
    private final Robot robot = Robot.getInstance();
    TelemetryData telemetryData;
    public GamepadEx driver;

    private Supplier<PathChain> pathChain;
    public GamepadEx operator;
    public String cosl;
    private ElapsedTime elapsedtime;
    private ElapsedTime elapsedtime2 = new ElapsedTime();
    private Pose testing = new Pose(97.79939209726444,97.28875379939208,Math.toRadians(45));
    boolean inFull = false;
    private Pose corner =  new Pose(8.620, 8.760,Math.toRadians(90));
    private Pose parkPose =  new Pose(121.0, 121.0, Math.toRadians(0));
    boolean autoParking = false;
    boolean shooting = false;
    boolean backup = false;
    boolean Intaking = false;
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
        for(LynxModule hub : robot.hubs){
            hub.setConstant(ALLIANCE_COLOR == "BLUE"? Color.BLUE : Color.RED);
        }
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
        elapsedtime2.startTime();
        elapsedtime.reset();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose,
                        parkPose
                )))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
    }

    @Override
    public void run() {
        robot.profiler.start("Full Loop");
        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3) {
            robot.intake.intake(Launcher.isFlapOpen == robot.launcher.flapOpen);
            Intaking = true;
        } else {
            robot.intake.intake(false);
            Intaking = false;
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
        if (driver.getButton(GamepadKeys.Button.DPAD_DOWN))
            autoPark();
        if (driver.getButton(GamepadKeys.Button.DPAD_UP))
            resetP();
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
        Launcher.activeControl = true;
        shooting = Intaking && Launcher.isFlapOpen && !robot.storage.emptyF;
        if(!autoParking)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        robot.updateLoop();
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        if(elapsedtime2.milliseconds()>1000)
            autoParking = false;
        robot.profiler.end("Full Loop");
    }
    public void resetP(){
        robot.follower.setPose(corner);
    }
    public void autoPark() {
        autoParking = true;
        elapsedtime2.reset();
        follower.followPath(pathChain.get());
    }
    @Override
    public void end() {
        robot.exportProfiler(robot.file);
    }
}
