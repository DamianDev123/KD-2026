package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem;

public class LedDriver extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    Servo led;
    public LedDriver(){
        led = robot.led;
    }

    @Override
    public void periodic(){

        robot.profiler.start("Led Loop");
        update();

        robot.profiler.end("Led Loop");
    }

    void update(){
        if(Storage.full){
            double error = Math.min(robot.launcher.errorAbs,100);
            double mapval = mapValue(error);
            led.setPosition(mapval);
        }else {
            led.setPosition(0.7);
        }
    }
    double mapValue(double x) {
        return 0.5 - 0.002 * x;
    }

}
