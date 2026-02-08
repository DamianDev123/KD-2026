package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp
public class testOp extends OpMode {
    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public MotorEx motor;
    public GamepadEx driver;

    // --- TUNING CONTROLS ---
    public static double forward = 0.42; // Your 0 degree position
    public static double right = 0.95;    // Your 90 degree position

    // Set this in Dashboard to any degree (e.g., 0, 90, 180, -45)
    public static double targetDegrees = 0.0;

    @Override
    public void init() {
        turretServo1 = new ServoEx(hardwareMap, "turretServo1")
                .setCachingTolerance(0.01);
        turretServo2 = new ServoEx(hardwareMap, "turretServo2")
                .setCachingTolerance(0.01);
        motor = new MotorEx(hardwareMap, "Shooter");
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        // 1. Find the "Range" (how many servo units represent 90 degrees)s
        double servoPosition =
                forward + (targetDegrees / 90.0) * (right - forward);
        turretServo1.set(servoPosition);
        turretServo2.set(servoPosition);

        // Telemetry for debugging
        telemetry.addData("Target Angle", targetDegrees);
        telemetry.addData("Calculated PWM", servoPosition);
        telemetry.update();
    }
}