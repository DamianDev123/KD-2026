package org.firstinspires.ftc.teamcode.Solvers.StateMachine;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Globals.Constants.autoInitialized;
import static org.firstinspires.ftc.teamcode.Globals.Constants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.Globals.Constants.redGoalPose;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;

import java.util.Objects;

@TeleOp
@Configurable
public class StateMachineTeleop extends CommandOpMode{

    Follower follower;

    boolean wantsToShoot = false;
    private final Robot robot = Robot.getInstance();
    public GamepadEx operator;
    public ActionRoutine actionRoutine;
    public MovementRoutine movementRoutine;

    private boolean prevTrigger;
    private boolean intakeActive;


    Runnable interruptAllStates = () -> {
        movementRoutine.nextState();
        actionRoutine.nextState();
    };
    @Override
    public void initialize() {
        operator = new GamepadEx(gamepad2);
        actionRoutine = new ActionRoutine();
        movementRoutine = new MovementRoutine(robot.follower,gamepad2,actionRoutine);
        Limelight.Companion.createFollower(hardwareMap);

        robot.turret.inAuto = false;
        robot.intake.runningAuto = false;

        setGoalPose();
        initializeFollower();
        lightUp();
        follower.update();
        robot.init(hardwareMap,follower);
    }

    private void handleIntake() {
        boolean trigger = operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;

        if (trigger && !prevTrigger) {
            actionRoutine.nextState(wantsToShoot ? "Shooting" : "Intaking");
            intakeActive = !wantsToShoot;
        }

        if (!trigger && prevTrigger && intakeActive) {
            actionRoutine.nextState();
            intakeActive = false;
        }

        prevTrigger = trigger;
    }

    @Override public void run(){
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> wantsToShoot = !wantsToShoot);
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Intake::intakeUp);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(movementRoutine::startGateSequence);
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(interruptAllStates);


        handleIntake();
        robot.updateLoop();
    }

    void initializeFollower(){
        if(!autoInitialized) {
            follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        }else {
            follower = robot.follower;
        }
        if(!autoInitialized){
            if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
                follower.setPose(robot.poses.getStartFromFar() .mirror());
            }else {
                follower.setPose(robot.poses.getStartFromFar());
            }
        }
    }
    void setGoalPose(){
        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            robot.GoalPose = blueGoalPose;
        }else {
            robot.GoalPose = redGoalPose;
        }
    }
    void lightUp(){
        for(LynxModule hub : robot.hubs){
            hub.setConstant(Objects.equals(ALLIANCE_COLOR, "BLUE") ? Color.BLUE : Color.RED);
        }
    }



}
