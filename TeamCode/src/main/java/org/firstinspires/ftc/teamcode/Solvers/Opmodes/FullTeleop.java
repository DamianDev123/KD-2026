package org.firstinspires.ftc.teamcode.Solvers.Opmodes;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Filter;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import android.graphics.Color;
import android.util.Log;

import java.util.Objects;

@TeleOp
@Configurable
public class FullTeleop extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();
    public GamepadEx driver;

    private Supplier<PathChain> pathChain;
    public GamepadEx operator;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.01,0,0,0);
    public PIDFController controller = new PIDFController(coefficients);
    public String cosl;
    private ElapsedTime elapsedtime;
    private ElapsedTime b1 = new ElapsedTime();
    private ElapsedTime b2 = new ElapsedTime();
    private ElapsedTime elapsedtime2 = new ElapsedTime();
    private Pose testing = new Pose(97.79939209726444,97.28875379939208,Math.toRadians(45));
    public static boolean inFull = false;
    public static double Por = -0.01;
    public static double BLUEOffset = 0.0001;
    public static double REDOffset= 0.0001;
    private Pose corner =  new Pose(8.620, 8.760,Math.toRadians(90));
    private Pose parkPose =  new Pose(121.0, 121.0, Math.toRadians(0));
    boolean autoParking = false;
    boolean shooting = false;
    boolean backup = false;
    boolean Intaking = false;
    Pose holdPointP = new Pose();
    boolean followerChanged = false;
    Filter loopTime = new Filter();
    @Override
    public void initialize() {
        robot.telemetry = telemetry;
        super.reset();
        shootingWhileMoving = true;
        if(!autoInitialized) {
            follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        }else {
            follower = robot.follower;
        }
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        robot.init(hardwareMap,follower);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        follower.startTeleOpDrive();
        for(LynxModule hub : robot.hubs){
            hub.setConstant(ALLIANCE_COLOR == "BLUE"? Color.BLUE : Color.RED);
        }
        cosl = ALLIANCE_COLOR;
        if(!autoInitialized){
            if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
                follower.setPose(robot.poses.getStartFromFar() .mirror());
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


        robot.intake.runningAuto = false;
        follower.update();
        Launcher.targetFlywheelVelocity = 0.0;
        elapsedtime = new ElapsedTime();
        elapsedtime2.startTime();
        b1.reset();
        b2.reset();
        robot.Preload = false;
        elapsedtime.reset();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose,
                        parkPose
                )))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
        robot.turret.inAuto = false;
        Limelight.Companion.createFollower(hardwareMap);
    }
    boolean intaking = false;
    boolean initialized = false;
    boolean setDown = false;
    @Override
    public void run() {

        Limelight.autoRunning = false;
        if(Limelight.followerCreated && !followerChanged){
           // follower = Limelight.follower;
            followerChanged = true;
        }
        robot.profiler.start("Full Loop");
        if(intaking && !initialized && !Storage.full)  {
            Intake.intakeDown();
            initialized = true;
        }else {
            initialized = false;
        }
        if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3) {
            intaking = true;
            //robot.launcher.setFlap(true);
            if(inFull){
                //if(robot.inZone())
                robot.launcher.setFlap(true);
                robot.intake.intake(robot.launcher.flapOpen);
            }else{
                robot.intake.intake(true);
            }
            Intaking = true;
        } else {
            intaking = false;
            robot.intake.intake(false);
            Intaking = false;
            robot.launcher.setFlap(false);
        }
        if (operator.getButton(GamepadKeys.Button.A)) {
            inFull = false;
        }
        if (inFull) {
            robot.turret.shouldAim = true;
            robot.launcher.doFlywheel = true;
            Intake.intakeDown();
            //robot.launcher.setFlap(true);
        }
        else {
            robot.turret.shouldAim = false;
            robot.launcher.doFlywheel = false;
            //robot.launcher.setFlap(false);
        }
        if(operator.getButton(GamepadKeys.Button.Y)){
            robot.follower.setHeading(Math.PI/2);
        }
        if (operator.getButton(GamepadKeys.Button.B)) {
            inFull = true;
        }
        if (operator.getButton(GamepadKeys.Button.A)) {
            robot.turret.inAuto = false;
        }
        Launcher.activeControl = true;
        shooting = Intaking && Launcher.isFlapOpen && !robot.storage.emptyF;
        double offset = REDOffset;
        if(ALLIANCE_COLOR == "BLUE")
            offset = BLUEOffset;
        controller.setCoefficients(coefficients);
        controller.setSetPoint(offset);
        double tx = robot.limelight.getTx();
        telemetry.addData("dd", tx);
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x+ (inFull&&robot.launcher.distance>40?controller.calculate(tx):0), true);

        if(operator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.storage.resetFull();
            Intake.intakeDownDown();
            setDown = true;
        } else if (setDown){
            Intake.intakeDown();
            setDown = false;
        }
        if(operator.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            Turret.offsetB += 1;
            Turret.offsetR += 1;
        }
        if(operator.getButton(GamepadKeys.Button.DPAD_LEFT)){
            Turret.offsetB -= 1;
            Turret.offsetR -= 1;
        }
        if(operator.getButton(GamepadKeys.Button.DPAD_UP)) {
            Launcher.distanceOffset += 1;
        }
        if(operator.getButton(GamepadKeys.Button.DPAD_DOWN)){
            Launcher.distanceOffset -= 1;
        }
        if(operator.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            Intake.intakeUp();
        robot.updateLoop();

        Log.i("Loop Times", String.valueOf(loopTime.updateFilteredVelocities(elapsedtime.milliseconds())));
        elapsedtime.reset();
        PanelsTelemetry.INSTANCE.getTelemetry().addData("zone",robot.inZone());
        if(elapsedtime2.milliseconds()>1000)
            autoParking = false;
        robot.tilt.set(driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        robot.profiler.end("Full Loop");
    }

    @Override
    public void end() {
        robot.exportProfiler(robot.file);
        shootingWhileMoving = false;
    }
}
