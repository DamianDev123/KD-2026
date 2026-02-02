package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
@TeleOp
public class testOp4 extends OpMode {
    public static double power1 = 0.0;
    public static double power2 = 0.0;

    public static double power3 = 0.0;

    public static double power4 = 0.0;
    public static double power5 = 0.0;

    public static double power6 = 0.0;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public MotorEx shooterMotor;
    public MotorEx shooterAMotor;

    public ServoEx hood;
    public ServoEx flap;
    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "Intake")
                .setCachingTolerance(0.01);
        transferMotor = new MotorEx(hardwareMap, "Transfer")
                .setCachingTolerance(0.01);
        shooterMotor = new MotorEx(hardwareMap, "Shooter")
                .setCachingTolerance(0.01);
        shooterAMotor = new MotorEx(hardwareMap, "shooterAssist")
                .setCachingTolerance(0.01);
        hood = new ServoEx(hardwareMap, "Hood").setCachingTolerance(0.001);
        flap = new ServoEx(hardwareMap, "Flap").setCachingTolerance(0.001);
    }

    @Override
    public void loop() {
        transferMotor.set(-power2);
        intakeMotor.set(power2);
        shooterMotor.set(power3);
        shooterAMotor.set(power4);
        hood.set(power5);
        flap.set(power6);
    }
}