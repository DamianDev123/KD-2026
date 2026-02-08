package org.firstinspires.ftc.teamcode.Solvers.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

public class ShootBalls extends CommandBase {
    private final Robot robot = Robot.getInstance();;
    ElapsedTime elapsedTime= new ElapsedTime();
    boolean initialized = false;
    int timeout = 2200;

    public ShootBalls(int i) {
        timeout = i;
    }
    public ShootBalls() {

    }

    @Override
    public void initialize() {
        elapsedTime.startTime();
        elapsedTime.reset();

        robot.turret.shouldAim = true;
        initialized = true;
    }
    public void reset(){
        initialized = true;

        robot.launcher.setFlap(true);
        robot.launcher.doFlywheel = true;
        Launcher.activeControl = true;
        elapsedTime.reset();

    }


    @Override
    public void execute() {
        if(!initialized)
            reset();
        robot.launcher.setFlap(true);
        robot.turret.shouldAim = true;
        robot.launcher.doFlywheel = true;
        Launcher.activeControl = true;
        if(((robot.launcher.inTolerance && Launcher.targetFlywheelVelocity!=0&&robot.turret.tolerable)|| Launcher.preload) && robot.launcher.flapOpen){
            robot.intake.intake(true);
            if(!initialized){
                elapsedTime.reset();
                initialized = true;
            }
        }

    }
    @Override
    public boolean isFinished() {
        return robot.storage.emptyF || elapsedTime.milliseconds()>5000;
    }


    @Override
    public void end(boolean inter) {
        initialized = false;
        robot.intake.intake(false);
        robot.launcher.setFlap(false);
        robot.launcher.doFlywheel = false;
    }
}