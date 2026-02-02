package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;
@Autonomous
public class FarzoneAuto extends CommandOpMode {
    Telemetry telemetry = new TelemetryImplUpstreamSubmission(this);
    Follower follower;
    private final Robot robot = Robot.getInstance();
    private Timer pathTimer, actionTimer, opmodeTimer;
    boolean shooting = false;
    Poses poses;
    private int pathState;
    ShootBalls shootBalls;
    @Override
    public void initialize() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Constants.OP_MODE_TYPE = Constants.OpModeType.AUTO;
        robot.init(hardwareMap,telemetry,follower);
        autoInitialized = true;

        if(ALLIANCE_COLOR ==  "BLUE"){
            follower.setPose(robot.poses.getStartFromFar().mirror());
        }else {
            follower.setPose(robot.poses.getStartFromFar());
        }

        if(ALLIANCE_COLOR == "BLUE"){
            robot.GoalPose = blueGoalPose;
        }else {
            robot.GoalPose = redGoalPose;
        }
        follower.update();
        poses = robot.poses;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shootBalls = new ShootBalls();
        if(ALLIANCE_COLOR == "BLUE"){
            poses.mirrorPaths();
        }

        poses.buildPaths();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!shooting){
                    shootBalls.schedule();
                    shooting = true;
                }
                if(shootBalls.isFinished()){

                    follower.followPath(poses.getToRow1());
                    shooting = false;
                    shootBalls.reset();
                    setPathState(1);
                }
                break;
            case 1:
                robot.launcher.setFlap(false);
                robot.intake.intake(true);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(poses.getRow1Intake(),true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    /* Score Preload */
                    robot.intake.intake(true);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(poses.getRow1Intake(),true);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    /* Score Preload */
                    robot.intake.intake(false);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(poses.toLaunch(follower)  ,true);
                    robot.launcher.setFlap(true);
                    setPathState(4);
                }
            case 4:

                Launcher.activeControl = true;
                if(!follower.isBusy()) {
                    follower.holdPoint(poses.getLaunchZone());
                    if(!shooting){
                        shootBalls.schedule();
                        shooting = true;
                    }
                    if(shootBalls.isFinished()){
                        follower.followPath(poses.getToRow2());
                        shooting = false;

                        robot.intake.intake(true);
                        setPathState(5);
                    }
                }
            case 5:
                if(!follower.isBusy()) {
                    robot.intake.intake(false);
                    //follower.holdPoint(poses.getLaunchZone());
                    Launcher.activeControl = false;
                }
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
