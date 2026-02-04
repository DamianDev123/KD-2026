package org.firstinspires.ftc.teamcode.Globals;
//
//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Scheduler;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.LedDriver;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

@Config
public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    public TelemetryData telemetryData;
    private static final Robot instance = new Robot();
    public Follower follower = null;
    public MotorGroup launchMotors;
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(15, 15);
    public Motor.Encoder launchEncoder;
    public Profiler profiler;
    public File file;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public Launcher launcher;
    public Intake intake;
    public Turret turret;
    public Poses poses;
    public ServoEx hoodServo;

    public ServoEx flapServo;
    public AprilTagProcessor aprilTag;
    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public CRServoGroup intakeSteps;
    public Motor.Encoder turretEncoder;
    public Pose GoalPose;
    public Pose CurrentPose;

    public Servo led;
    public LedDriver ledDriver;
    public Storage storage;
    public ShootingWhileMoving shootingWhileMoving;
    public LynxModule ex;
    public List<LynxModule> hubs;
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensor = null;

    public static Robot getInstance() {
        return instance;
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(HardwareMap hwMap, Telemetry telemetry, Follower _follower){
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();
        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        hubs = allHubs;
        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false) // Log EVERYTHING
                .build();
        ex = getExpansionHub(allHubs);
        ex.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = _follower;
        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            GoalPose = blueGoalPose;
        }else {
            GoalPose = redGoalPose;
        }

        telemetryData = new TelemetryData(telemetry);

        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "Shooter")
                        .setCachingTolerance(0.01)
                        .setInverted(true),
                new MotorEx(hwMap, "shooterAssist")
                        .setCachingTolerance(0.01)

        );
        launchMotors.setRunMode(Motor.RunMode.RawPower);
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launchEncoder = new Motor(hwMap, "frontRight").encoder;
        launchEncoder.setDirection(Motor.Direction.FORWARD);
        intakeMotor = new MotorEx(hwMap, "Intake")
                        .setCachingTolerance(0.01);
        transferMotor = new MotorEx(hwMap, "Transfer")
                        .setCachingTolerance(0.01);
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor.setInverted(true);
        transferMotor.setRunMode(Motor.RunMode.RawPower);
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        led = hwMap.get(Servo.class,"led");
        hoodServo = new ServoEx(hwMap, "Hood").setCachingTolerance(0.001);
        flapServo = new ServoEx(hwMap, "Flap").setCachingTolerance(0.001);

        turretServo1 = new ServoEx(hwMap, "turretServo1")
                        .setCachingTolerance(0.01);
        turretServo2 = new ServoEx(hwMap, "turretServo2")
                 .setCachingTolerance(0.01);

        turretEncoder = new Motor(hwMap, "Intake").encoder;
        intakeSteps = new CRServoGroup(
                new CRServoEx(hwMap, "intake1").setInverted(true),
                new CRServoEx(hwMap, "intake2")
        );
        intake = new Intake();
        launcher = new Launcher();
        turret = new Turret();
        poses = new Poses();
        storage = new Storage();
        storage.stage1 = hwMap.get(DigitalChannel.class, "S2");
        storage.stage2 = hwMap.get(DigitalChannel.class, "S1");
        storage.stage3 = hwMap.get(DigitalChannel.class, "S3");
        ledDriver = new LedDriver();
        if(ALLIANCE_COLOR == "BLUE"){
            CurrentPose = poses.getStartFromFar().mirror();
        }else {
            CurrentPose = poses.getStartFromFar();
        }
        shootingWhileMoving = new ShootingWhileMoving();
        voltageSensor = hwMap.voltageSensor;
        //poses.buildPaths();
    }

    public Pose getPose(){
        return CurrentPose;
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double getVoltage() {
        return 12/getBatteryVoltage();
    }
    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("An error occurred", e.toString());
                Log.e(e.toString(), Arrays.toString(e.getStackTrace()));
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }
    @NonNull
    public static LynxModule getExpansionHub(List<LynxModule> ll) {
        ll.removeIf((hub1)->{return hub1.isParent() ||String.valueOf(hub1.getRevProductNumber()).startsWith("1");});
        return ll.get(0);
    }
    public void updateLoop() {
        profiler.start("Update Loop");
        ex.clearBulkCache();

        profiler.start("Subsystem Loop");
            CommandScheduler.getInstance().run();
            Scheduler.getInstance().periodic();
        profiler.end("Subsystem Loop");
            telemetryData.update();
            follower.update();
            CurrentPose = follower.getPose();
            Drawing.drawDebug(follower);
            Drawing.drawRobot(GoalPose, "#4CAF50");
            Drawing.drawRobot(ShootingWhileMoving.predictedPose, "#4CAF50");
            robotZone.setPosition(CurrentPose.getX(), CurrentPose.getY());
            robotZone.setRotation(CurrentPose.getHeading());

        dashboardTelemetry.addData("Pase", CurrentPose);
        dashboardTelemetry.update();

        profiler.end("Update Loop");
      //  Drawing.drawRobot(new Pose(follower.getPose().getY(),follower.getPose().getX()),"#3F51B5");
    }
    public boolean inZone(){
        return robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone);
    }
    public boolean nearZone(){
        double distanceToClose = robotZone.distanceTo(closeLaunchZone);
        double x = robotZone.distanceTo(farLaunchZone);
        if(distanceToClose<x){
            x=distanceToClose;
        }
        return x<NearDistance;
    }
}
