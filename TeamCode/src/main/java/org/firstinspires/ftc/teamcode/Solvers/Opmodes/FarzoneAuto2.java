package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Autonomous
public class FarzoneAuto2 extends CommandOpMode {
    Telemetry telemetry = new TelemetryImplUpstreamSubmission(this);
    Follower follower;
    private final Robot robot = Robot.getInstance();
    private Timer pathTimer, opmodeTimer;
    boolean shooting = false;
    private int pathState;
    ShootBalls shootBalls;

    public PathChain toRow1,toRow1Intake,toLaunchZoneRow1,toRow2,toRow2Intake,toGate,toLaunchZoneRow2,toRow3,toRow3Intake,toLaunchZoneRow3,toPark;

    private Pose startPose = new Pose(96, 8, Math.toRadians(90));
    private Pose row1 = new Pose(106.587, 34.933);
    private Pose row1End = new Pose(132.480, 35.187);
    private Pose row2 = new Pose(108.933, 59.253);
    private Pose row2End = new Pose(132.053, 59.720);
    private Pose gate = new Pose(129.240, 70.693);
    private Pose row3 = new Pose(107.653, 83.787);
    private Pose row3End = new Pose(128.640, 83.827);
    private Pose park =  new Pose(96.000, 20.000);


    private final List<Pose> controlPoints = new ArrayList<>(List.of(
            new Pose(94.387, 34.053),
            new Pose(107.760, 24.567),
            new Pose(96.520, 59.867),
            new Pose(122.340, 66.780),
            new Pose(94.780, 47.827),
            new Pose(90.333, 81.200),
            new Pose(107.760, 24.567)
    ));
    private double intakeHeading = 0.0;
    public void mirrorPaths(){
        startPose = startPose.mirror();
        row1 = row1.mirror();
        row1End = row1End.mirror();
        row2 = row2.mirror();
        row2End = row2End.mirror();
        gate = gate.mirror();
        row3 = row3.mirror();
        row3End = row3End.mirror();
        park = park.mirror();

        for (int i = 0; i < controlPoints.size(); i++) {
            controlPoints.set(i, controlPoints.get(i).mirror());
        }

        intakeHeading = 180;
    }

    public void buildPaths() {
        toRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                controlPoints.get(0),
                                row1
                        )
                ).setTangentHeadingInterpolation()

                .build();

        toRow1Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                                row1,
                                row1End
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();

        toLaunchZoneRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row1End,
                                controlPoints.get(1),
                                startPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(90))

                .build();
        toRow2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                controlPoints.get(2),
                                row2
                        )
                ).setTangentHeadingInterpolation()

                .build();

        toRow2Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                                row2,
                                row2End
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();

        toGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row2End,
                                controlPoints.get(3),
                                gate
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(intakeHeading))

                .build();
        toLaunchZoneRow2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gate,
                                controlPoints.get(4),
                                startPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(90))

                .build();
        toRow3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                               startPose,
                                controlPoints.get(5),
                                row3
                        )
                ).setTangentHeadingInterpolation()

                .build();

        toRow3Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                                row3,

                               row3End
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();

        toLaunchZoneRow3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row3End,
                                controlPoints.get(6),
                                startPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(90))

                .build();
        toPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                park
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }


    @Override
    public void initialize() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Constants.OP_MODE_TYPE = OpModeType.AUTO;
        robot.init(hardwareMap,telemetry,follower);
        autoInitialized = true;

        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            follower.setPose(robot.poses.getStartFromFar().mirror());
        }else {
            follower.setPose(robot.poses.getStartFromFar());
        }

        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            robot.GoalPose = blueGoalPose;
        }else {
            robot.GoalPose = redGoalPose;
        }
        follower.update();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shootBalls = new ShootBalls();
        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            mirrorPaths();
        }
        buildPaths();
        shootBalls.schedule();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (shootBalls.isFinished()) {
                    follower.followPath(toRow2);
                    setPathState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.intake.intake(true);
                    follower.followPath(toRow2Intake, true);
                    setPathState(2);
                }
                break;
            case 2:
                robot.intake.intake(false);
                if (!follower.isBusy()) {
                    follower.followPath(toGate, true);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(toLaunchZoneRow2, true);
                    robot.launcher.setFlap(true);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy()) {
                    if (!shooting) {
                        shootBalls.schedule();
                        shooting = true;
                    }
                    if (shootBalls.isFinished()) {
                        follower.followPath(toRow1, true);
                        shooting = false;
                        setPathState(5);
                    }
                }
            case 5:
                robot.intake.intake(true);
                if (!follower.isBusy()) {
                    follower.followPath(toRow1Intake, true);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    robot.intake.intake(false);
                    follower.followPath(toLaunchZoneRow1, true);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy()) {
                    if (!shooting) {
                        shootBalls.schedule();
                        shooting = true;
                    }
                    if (shootBalls.isFinished()) {
                        follower.followPath(toRow3, true);
                        shooting = false;
                        setPathState(8);
                    }
                }
            case 8:
                robot.intake.intake(true);
                if (!follower.isBusy()) {
                    follower.followPath(toRow3Intake, true);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    robot.intake.intake(false);
                    follower.followPath(toLaunchZoneRow3, true);
                    setPathState(10);
                }
            case 10:
                if(!shooting) {
                    shootBalls.schedule();
                    shooting = true;
                }
                if(shootBalls.isFinished()) {
                    follower.followPath(toPark, true);
                    shooting = false;
                    setPathState(11);
                }
            case 11:
                follower.holdPoint(park);
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void run(){
        robot.updateLoop();
        autonomousPathUpdate();
    }
}
