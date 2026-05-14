package org.firstinspires.ftc.teamcode.Solvers.StateMachine;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.State;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.StateMachine;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.TimedState;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;

public class ActionRoutine implements StateMachine {
    private final Robot robot = Robot.getInstance();
    public ActionRoutine() {
        initStateMachine();
        Launcher.activeControl = true;
    }
    @State(first = true)
    public void Standby(){
        Intake.intakeUp();
    }
    @TimedState(time = 3.0, next = "Reset")
    public void Shooting(){
        Intake.intakeDown();
        robot.turret.shouldAim = true;
        robot.launcher.doFlywheel = true;
        robot.launcher.setFlap(true);
        robot.intake.intake(robot.launcher.flapOpen);
    }
    @State void Intaking(){
        Intake.intakeDown();
        robot.intake.intake(true);
        if(Storage.full)
            nextState();

    }
    @State void Reset(){
        robot.turret.shouldAim = false;
        robot.launcher.doFlywheel = false;
        robot.launcher.setFlap(false);
        nextState();
    }
}
