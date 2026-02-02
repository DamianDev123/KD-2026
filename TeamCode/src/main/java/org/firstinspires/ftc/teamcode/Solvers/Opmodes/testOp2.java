package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
@TeleOp
public class testOp2 extends OpMode {
    public DigitalChannel stage1;
    public DigitalChannel stage2;

    public DigitalChannel stage3;
    public static double targetDegrees = 0.0;

    @Override
    public void init() {
        stage1 = hardwareMap.get(DigitalChannel.class, "S2");
        stage2 = hardwareMap.get(DigitalChannel.class, "S1");
        stage3 = hardwareMap.get(DigitalChannel.class, "S3");
    }

    @Override
    public void loop() {


        // Telemetry for debugging
        telemetry.addData("Target Angle1", stage1.getState());
        telemetry.addData("Target Angle2", stage2.getState());
        telemetry.addData("Target Angle3", stage3.getState());
        telemetry.update();
    }
}