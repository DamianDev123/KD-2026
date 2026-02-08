package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    ElapsedTime elapsedTime2 = new ElapsedTime();

    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedTime3 = new ElapsedTime();
    Boolean checkedForCurrent = false;
    public static double CurrentThreshold = 6.0;
    public static double tolerance = 50;
    public BooleanSupplier supplier = () -> Storage.full;
    Double current = 0.0;
    boolean lastResponse = true;
    boolean overloaded = false;
    public boolean full = false;
    public boolean startClear = false;
    public ElapsedTime elapsedTime4 = new ElapsedTime();

    public Intake() {
        elapsedTime.startTime();
        elapsedTime2.startTime();

    }

    public void intake(Boolean onoff) {
            if (Launcher.isFlapOpen) {
                robot.transferMotor.set(onoff ? 1 : 0);
                robot.intakeMotor.set(onoff ? -1 : 0);
            }
            robot.transferMotor.set((elapsedTime4.milliseconds()>200 || Launcher.isFlapOpen) && onoff ? 1 : 0);
            robot.intakeMotor.set(onoff ? -1 : 0);


            if (full && Launcher.isFlapOpen && onoff) {
                if (!startClear) {
                    elapsedTime3.startTime();
                    elapsedTime3.reset();
                }
                startClear = true;
            }

    }

    @Override
    public void periodic() {
        update();
        if((robot.storage.list[1] && robot.storage.list[2]))
            elapsedTime4.reset();
    }
    void update(){
        if (Launcher.isFlapOpen) {
            overloaded = false;
        }
        if (startClear) {
            if (elapsedTime3.milliseconds() > 700) {
                startClear = false;
                full = false;
            }
        }
    }


    boolean canPushTransfer() {
        if (Launcher.isFlapOpen) {
            elapsedTime2.reset();
            return true;
        }
        if (overloaded) {
            return false;
        }
        if (!checkedForCurrent || elapsedTime.milliseconds() > 100) {
            current = robot.transferMotor.getCurrent(CurrentUnit.AMPS);
            if (!checkedForCurrent) {
                elapsedTime.reset();
            }
        }
        if (current > CurrentThreshold) {
            elapsedTime2.reset();
            overloaded = true;
            return false;
        }

        if (lastResponse || elapsedTime2.milliseconds() > 200) {
            elapsedTime2.reset();
            overloaded = false;
            return true;
        }

        elapsedTime2.reset();
        overloaded = true;
        return false;
    }


}

