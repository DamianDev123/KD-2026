package org.firstinspires.ftc.teamcode.Solvers.StateMachine;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5.cp;
import static org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5.gatein;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.State;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.StateMachine;

import java.util.Objects;

public class MovementRoutine implements StateMachine {
    Follower follower;
    Gamepad gamepad;
    ActionRoutine actionRoutine;
    private Supplier<PathChain> toGate;
    private Supplier<PathChain> gateIn;
    private Supplier<PathChain> toLaunch;
    private Pose launch = new Pose(76.203, 79.630);
    MovementRoutine(Follower follower, Gamepad gamepad, ActionRoutine actionRoutine){
        this.follower = follower;
        this.gamepad = gamepad;
        this.actionRoutine = actionRoutine;


        toGate = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, CloseZoneAuto5.gate)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(30), 0.8))
                .build();
        gateIn = () -> follower.pathBuilder().addPath(
                        new BezierCurve(
                                follower::getPose,
                                cp,
                                gatein
                        )
                ).setLinearHeadingInterpolation(
                        Math.toRadians(30),
                        Math.toRadians(30))
                .build();
        toLaunch = () -> follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,
                                launch
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        initStateMachine();

    }
    @State(first = true)
    public void driving(){
        if(justEntered())
            follower.startTeleopDrive();
        follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
        );
    }
    public void startGateSequence(){
        nextState("toGateM");
    }
    @State()
    public void toGateM(){
        follower.followPath(toGate.get());
        if(follower.getCurrentTValue()>0.9)
            nextState("toGateInM");
    }
    @State()
    public void toGateInM(){
        actionRoutine.nextState("Intaking");
        follower.followPath(gateIn.get());
        if(follower.getCurrentTValue()>0.9)
            nextState();
    }
    @State()
    public void toLaunchM(){
        follower.followPath(toLaunch.get());
        if(follower.getCurrentTValue()>0.9)
            nextState();
    }
}
