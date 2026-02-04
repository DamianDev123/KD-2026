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
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;

import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class CloseZoneAuto5 extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toLaunch,
            toRow1,toLaunchZoneRow1,
            toRow2,toLaunchZoneRow2,
            toRow3,toLaunchZoneRow3,
            toGate,toLaunchGate,
            toLaunchGate2;

    /* ------------------- Base Poses ------------------- */

    private Pose startPose   = new Pose(121.0, 121.0, Math.toRadians(0));
    private Pose launchZone  = new Pose(83.5 , 75.0);
    private Pose launchZone2  = new Pose(93 , 78);
    private Pose row1        = new Pose(130.000, 36.500);
    private Pose row2        = new Pose(130.000, 59.500);
    private Pose row3        = new Pose(120.000, 83.500);
    private Pose gate        = new Pose(136, 63);

    /* ------------------- Control Points ------------------- */

    private Pose cpRow3Out = new Pose(93.267, 83.700);
    private Pose cpGateMid = new Pose(112.473, 67.707);

    private Pose cpRow2Out = new Pose(101.43, 57.287);
    private Pose cpRow2In  = new Pose(101.43, 57.287);

    private Pose cpRow1Out = new Pose(90.447, 31.963);
    private Pose cpRow1In  = new Pose(85.000, 31.000);
    private Pose cpGateIn  =  new Pose(113.35076923076922, 57);

    private double intakeHeading = 0.0;
    private double gateHeading = 20.0;

    /* ------------------- Alliance Mirroring ------------------- */

    public void mirrorPaths() {
        startPose  = startPose.mirror();
        launchZone = launchZone.mirror();
        row1       = row1.mirror();
        row2       = row2.mirror();
        row3       = row3.mirror();
        gate       = gate.mirror();

        cpRow3Out = cpRow3Out.mirror();
        cpGateMid = cpGateMid.mirror();
        cpRow2Out = cpRow2Out.mirror();
        cpRow2In  = cpRow2In.mirror();
        cpRow1Out = cpRow1Out.mirror();
        cpRow1In  = cpRow1In.mirror();
        cpGateIn = cpGateIn.mirror();

        intakeHeading = mirrorAngle(intakeHeading);
        gateHeading = mirrorAngle(gateHeading);
        robot.GoalPose = blueGoalPose;
    }
    public double mirrorAngle(double angle){
        return -angle+180;
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
                new BezierLine(launchZone2, row3)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toLaunchZoneRow3 = follower.pathBuilder().addPath(
                new BezierLine(row3, launchZone2)
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


        toLaunchZoneRow2 = follower.pathBuilder().addPath(
                new BezierCurve(row2, cpRow2In, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(gateHeading)
        ).build();

        toRow1 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone2,cpRow1In, row1)
        ).setLinearHeadingInterpolation(
                Math.toRadians(-64),
                Math.toRadians(-20)
        ).build();

        toLaunchZoneRow1 = follower.pathBuilder().addPath(
                new BezierLine(row1, launchZone)
        ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        toGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        launchZone,
                        cpGateIn,
                        gate
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(gateHeading)
        ).build();

        toLaunchGate = follower.pathBuilder().addPath(
                new BezierLine(
                        gate,
                        launchZone
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(gateHeading),
                Math.toRadians(gateHeading)
        ).build();
        toLaunchGate2 = follower.pathBuilder().addPath(
                new BezierLine(
                      gate,
                        launchZone2
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(gateHeading),
                Math.toRadians(intakeHeading)
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

                new InstantCommand(() -> Constants.shootingWhileMoving = true),
                new FollowPathCommand(follower, toLaunch, true).alongWith(new ShootBalls()),
                new InstantCommand(() -> robot.intake.intake(true)),

                new InstantCommand(() -> Constants.shootingWhileMoving = false),
                new FollowPathCommand(follower, toRow2).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower, toLaunchZoneRow2, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false)),
                                new InstantCommand(() -> robot.launcher.setFlap(true))
                        ).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),


                new FollowPathCommand(follower, toGate, true).withTimeout(3000),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull(),
                new FollowPathCommand(follower, toLaunchGate, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false))
                        ).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),


                new FollowPathCommand(follower, toGate, true).withTimeout(3000),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull(),
                new FollowPathCommand(follower, toLaunchGate, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false))
                        ).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),

                new FollowPathCommand(follower, toGate, true).withTimeout(3000),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull(),
                new FollowPathCommand(follower, toLaunchGate2, true)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false))
                        ).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),

                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow3).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,toLaunchZoneRow3, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new ShootBalls(),

                new InstantCommand(()-> robot.turret.toggle =false)
        );

        schedule(autonomousSequence);
    }
    public BooleanSupplier nearEnd = ()-> follower.getCurrentTValue()>0.7;
    @Override
    public void run() {
        super.run();
        robot.updateLoop();

    }

    @Override
    public void end() {
        robot.turret.setRunningAuto(false);
        robot.turret.shouldAim = false;
    }
}
