package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Globals.Constants.OpModeType;
import static org.firstinspires.ftc.teamcode.Globals.Constants.autoInitialized;
import static org.firstinspires.ftc.teamcode.Globals.Constants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.Globals.Constants.redGoalPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.WaitFull;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;

import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class CloseZoneAuto4 extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toLaunch,
            toRow1, toRow1Intake, toLaunchZoneRow1,
            toRow2, toRow2Intake, toLaunchZoneRow2,
            toRow3, toRow3Intake, toLaunchZoneRow3,
            toGate, tolaunchGate;

    /* ------------------- Base Poses ------------------- */

    private Pose startPose   = new Pose(121.0, 121.0, Math.toRadians(0));
    private Pose launchZone  = new Pose(86.000, 98.000);
    private Pose row1        = new Pose(130.000, 36.500);
    private Pose row1End     = new Pose(120.500, 35.500);
    private Pose row2        = new Pose(130.000, 59.500);
    private Pose row2End     = new Pose(128.500, 59.500);
    private Pose row3        = new Pose(120.000, 83.500);
    private Pose row3End     = new Pose(128.500, 83.500);
    private Pose gate        = new Pose(110.000, 70.640);

    /* ------------------- Control Points ------------------- */

    private Pose cpRow3Out = new Pose(93.267, 83.700);
    private Pose cpGateMid = new Pose(112.473, 67.707);

    private Pose cpRow2Out = new Pose(93.153, 60.137);
    private Pose cpRow2In  = new Pose(110.303, 58.537);

    private Pose cpRow1Out = new Pose(90.447, 31.963);
    private Pose cpRow1In  = new Pose(85.000, 31.000);

    private double intakeHeading = 0.0;

    /* ------------------- Alliance Mirroring ------------------- */

    public void mirrorPaths() {
        startPose  = startPose.mirror();
        launchZone = launchZone.mirror();
        row1       = row1.mirror();
        row1End    = row1End.mirror();
        row2       = row2.mirror();
        row2End    = row2End.mirror();
        row3       = row3.mirror();
        row3End    = row3End.mirror();
        gate       = gate.mirror();

        cpRow3Out = cpRow3Out.mirror();
        cpGateMid = cpGateMid.mirror();
        cpRow2Out = cpRow2Out.mirror();
        cpRow2In  = cpRow2In.mirror();
        cpRow1Out = cpRow1Out.mirror();
        cpRow1In  = cpRow1In.mirror();

        intakeHeading = 180;
        robot.GoalPose = blueGoalPose;
    }

    /* ------------------- Path Building ------------------- */

    public void buildPaths() {

        toLaunch = follower.pathBuilder().addPath(
                new BezierLine(startPose, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow3 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow3Out, row3)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow3Intake = follower.pathBuilder().addPath(
                new BezierLine(row3, row3End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow3 = follower.pathBuilder().addPath(
                new BezierCurve(row3, cpGateMid, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow2 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow2Out, row2)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow2Intake = follower.pathBuilder().addPath(
                new BezierLine(row2, row2End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow2 = follower.pathBuilder().addPath(
                new BezierCurve(row2End, cpRow2In, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow1 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow1Out, row1)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow1Intake = follower.pathBuilder().addPath(
                new BezierLine(row1, row1End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow1 = follower.pathBuilder().addPath(
                new BezierCurve(row1End, cpRow1In, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        launchZone,
                        new Pose(100.370, 50.507),
                        new Pose(137, 60)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(30)
        ).build();

        tolaunchGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(138.412, 59.319),
                        new Pose(96.496, 52.888),
                        launchZone
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(30),
                Math.toRadians(0)
        ).build();
    }

    /* ------------------- Init ------------------- */

    @Override
    public void initialize() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Constants.OP_MODE_TYPE = OpModeType.AUTO;

        robot.init(hardwareMap, telemetry, follower);
        autoInitialized = true;

        if (Objects.equals(ALLIANCE_COLOR, "BLUE")) {
            mirrorPaths();
            follower.setPose(startPose);
        } else {
            follower.setPose(startPose);
            robot.GoalPose = redGoalPose;
        }

        shootBalls = new ShootBalls();
        shootBalls.initialize();

        robot.turret.setRunningAuto(true);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> robot.turret.shouldAim = true),
                new InstantCommand(() -> robot.launcher.setFlap(true)),
                new FollowPathCommand(follower, toLaunch, true).alongWith(new ShootBalls(3000)),

                new InstantCommand(() -> robot.intake.intake(true)),
                new FollowPathCommand(follower, toRow2).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower, toLaunchZoneRow2, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false)),
                                new InstantCommand(() -> robot.launcher.setFlap(true))
                        ),
                new ShootBalls(),

                new FollowPathCommand(follower, toGate, true).withTimeout(3000),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull(),
                new FollowPathCommand(follower, tolaunchGate, true)
                        .halfWay(
                        new InstantCommand(() -> robot.intake.intake(false))
                ),
                new ShootBalls(),

                new FollowPathCommand(follower, toGate, true).withTimeout(3000),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull(),
                new FollowPathCommand(follower, tolaunchGate, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false))
                        ).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),

                new InstantCommand(()-> robot.turret.toggle =false),
                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow3).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,toLaunchZoneRow3, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new ShootBalls()
        );

        schedule(autonomousSequence);
    }
    public BooleanSupplier nearEnd = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return follower.getCurrentTValue()>0.4 ;
        }
    };
    @Override
    public void run() {
        super.run();
        robot.updateLoop();
    }

    @Override
    public void end() {
        robot.turret.setRunningAuto(true);
        robot.turret.shouldAim = false;
    }
}
