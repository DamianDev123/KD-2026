package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem;

public class Storage extends Subsystem {
    public static boolean full = false;
    public boolean contains = false;
    public boolean[] list = {true,true,true};
    public DigitalChannel stage1 = null;
    public DigitalChannel stage2 = null;

    public DigitalChannel stage3 = null;
    ElapsedTime timer = new ElapsedTime();
    public boolean emptyF = false;
    public Storage() {
        timer.startTime();
        timer.reset();
    }
    void update(){

        if(stage1==null) return;
        list[0] = !stage1.getState();
        list[1] = !stage2.getState();
        list[2] = !stage3.getState();
        full = list[0] && list[1] && list[2];
        contains = list[0] || list[1] || list[2];
        if(contains)
            timer.reset();
        emptyF = timer.milliseconds()>100;
    }
    @Override
    public void periodic() {
        update();
    }
}
