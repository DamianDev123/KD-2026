package org.firstinspires.ftc.teamcode.Globals;
//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import dev.nextftc.control.feedback.PIDCoefficients;


@Config
public class Constants {
    public static OpModeType OP_MODE_TYPE;
    public enum ZoneType {
        Farzone(1), Closezone(-1);

        private int val;

        ZoneType(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public static Pose redGoalPose = new Pose(138,138, 0.0);
    public static Pose redTurretGoalPose = new Pose(128,128, 0.0);
    public static double redX = 138; // Degrees from horizontal //
    public static double redY = 138; // MUST MATCH WITH VALUE ABOVE
    public static Pose blueGoalPose = redGoalPose.mirror();
    public static String ALLIANCE_COLOR = "RED";
    public static ZoneType zoneType = ZoneType.Farzone;
    public static double LAUNCHER_DEFAULT_ON_SPEED = 0.7; // Power
    public static double MIN_HOOD_ANGLE = 18.188; // Degrees from horizontal //
    public static double MIN_HOOD_SERVO_POS = 1; // MUST MATCH WITH VALUE ABOVE
    public static double MAX_HOOD_ANGLE = 44.188; // Degrees from horizontal //
    public static double MAX_HOOD_SERVO_POS = 0; // Position // MUST MATCH WITH VALUE ABOVE
    public static double LAUNCHER_MAX_VELOCITY = 1900; // Ticks/second
    public static double LAUNCHER_MIN_VELOCITY = 900; // Ticks/second

    public static final double GRAVITY = 9.81; // meters/second
    public static double LAUNCHER_HEIGHT =10.5*DistanceUnit.mPerInch; // meters // 13 inches
    public static  double TARGET_HEIGHT =1; // meters
    public static double LAUNCHER_MAX_BALL_VELOCITY = 12; // Meters/second // TODO: tune this to potentially be lower
    public static double GOAL_LIP = 1; // Meters
    public static double BACKBOARD_Y_OFFSET = 0.2; // Meters
    public static double LIP_BUFFER = 0.1; // Meters

    public static boolean autoInitialized = false;
    public static boolean shootingWhileMoving = false;
    public static double MAX_DRIVE_VELOCITY = 7.75 * 12; // Inches/second
    public static Double NearDistance = 5.0;
    public static PIDCoefficients TURRET_PIDF_COEFFICIENTS= new PIDCoefficients(0.0002,0,0);
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
