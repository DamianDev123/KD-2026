package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import static org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShooterConstants.SCORE_HEIGHT;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Globals.MathFunctions;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.Lut;

import java.util.Arrays;
import java.util.List;

public class ShootingWhileMoving extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public static Pose predictedPose = new Pose();

    public Pose predictedPose2 = new Pose();
    public static double Weight = 0.6;
    public static double Weight2 = 0.6;
    public static Vector robotGoalVector = new Vector();
    Vector robotVelocity = new Vector();
    Pose currentPose = new Pose();

    public static Vector p = new Vector();
    public static Pose goalP = new Pose();
    public double[] calcs = new double[] {0,0};
//    private static final List<Double> launcherInput  = Arrays.asList(-0.01, 0.0, 231.0,   207.0,   226.0,   186.0,  222.0,   165.0); // input: velocity (m/s)
//    private static final List<Double> launcherOutput = Arrays.asList(-0.01, 0.0, 1400.0,  1000.0,  1200.0,  900.0,  1100.0,    800.0); // output: ticks/s
//
//    private static final Lut launcherLUT = new Lut(
//            launcherInput,
//            launcherOutput,
//            true
//    );
//
//    private static final Lut inverseLauncherLUT = new Lut(
//            launcherOutput,
//            launcherInput,
//            true
//    );
//    public ShootingWhileMoving(){
//
//        launcherLUT.createLUT();
//        inverseLauncherLUT.createLUT();
//    }

    public void update2() {
        try {
            robotVelocity = robot.follower.getVelocity();
            currentPose = robot.getPose();
            double x = robot.launcher.distance * DistanceUnit.mPerInch;
            double lVel = Launcher.targetFlywheelVelocityIn * DistanceUnit.mPerInch;
            double tof = org.firstinspires.ftc.teamcode.Globals.MathFunctions.getTimeOfFlight(x * DistanceUnit.mPerInch, lVel * DistanceUnit.mPerInch, Launcher.targetHoodAngle);
            Vector vel = robotVelocity;
            double angular = robot.follower.getAngularVelocity();
            robot.telemetryData.addData("tof", tof);
            predictedPose = new Pose(
                    currentPose.getX()
                            + vel.getXComponent() * tof,
                    //+ 0.5 * accel.getXComponent() * t1 * t1,

                    currentPose.getY()
                            + vel.getYComponent() * tof,
                    currentPose.getHeading()+angular*tof
                    //+ 0.5 * accel.getYComponent() * t1 * t1
            );
        }catch (Exception e){
            return;
        }

        //update();
    }
    @Override
    public void periodic(){

        robot.profiler.start("Moving Loop");
        currentPose = robot.CurrentPose;
        Pose g = robot.GoalPose;
        if(Constants.shootingWhileMoving) {

            double angular = robot.follower.getAngularVelocity();
            robotVelocity = robot.follower.getVelocity();
            double x = dista(currentPose) * DistanceUnit.mPerInch;
            double tof = org.firstinspires.ftc.teamcode.Globals.MathFunctions.getTimeOfFlight(x, Math.max(Launcher.targetFlywheelVelocityIn * DistanceUnit.mPerInch, 2), Launcher.targetHoodAngle) + 0.4;
            Vector vel = robotVelocity;
            vel.setMagnitude(tof);
            goalP = new Pose(
                    g.getX() - vel.getXComponent()*tof,
                    g.getY()-vel.getYComponent()*tof
            );

        }else {
            goalP = g;
        }
        double dist = dista(goalP);


        calcs = org.firstinspires.ftc.teamcode.Globals.MathFunctions.distanceToLauncherValues(dist * DistanceUnit.mPerInch);
        robot.profiler.end("Moving Loop");
    }
    public double dista(Pose p){
        double dx = p.getX() - currentPose.getX();
        double dy = p.getY() - currentPose.getY();
        if(Launcher.preload){
            dx = p.getX() - Launcher.preloadPos.getX();
            dy = p.getY() - Launcher.preloadPos.getY();
        }

        return Math.sqrt(dx * dx + dy * dy);
    }

//    void update(){
//        currentPose = robot.CurrentPose;
//        robotVelocity = robot.follower.getVelocity();
//        Vector accel = robot.follower.getAcceleration();
//        currentPose = robot.CurrentPose;
//        Vector vel = robotVelocity;
//        double t1 = Weight;
//        double t2 = Weight2;
//        double headingV = robot.follower.getAngularVelocity();
//        robot.telemetryData.addData("accel",headingV);
//
//        predictedPose = new Pose(
//                currentPose.getX()
//                        + vel.getXComponent() * t1,
//                //+ 0.5 * accel.getXComponent() * t1 * t1,
//
//                currentPose.getY()
//                        + vel.getYComponent() * t1
//                //+ 0.5 * accel.getYComponent() * t1 * t1
//        );
//
//        predictedPose2 = new Pose(
//                currentPose.getX()
//                        + vel.getXComponent() * t2
//                        +( + 0.5 * accel.getXComponent() * (t2 * t2) *accelScale),
//
//                currentPose.getY()
//                        + vel.getYComponent() * t2
//                        + (0.5 * accel.getYComponent() * (t2 * t2) *accelScale),
//                currentPose.getHeading() + headingV*t2*headingScale
//        );
//        robotGoalVector = robot.GoalPose.getAsVector().minus(predictedPose.getAsVector());
//
//        double dist = robotGoalVector.getMagnitude();
//        double[] calcs;
//        if(dist>110)
//            calcs = getShootingFar(robotGoalVector.getMagnitude());
//        else
//            calcs = getShootingParams(robotGoalVector.getMagnitude());
//        targetVelIn = calcs[0];
//        targetVelTicks = getFlywheelTicksFromVelocity(targetVelIn);
//        targetHoodAngle = calcs[1];
//        calcPoses = calculateShotVector(currentPose.getHeading());
//    }
//
//    private double[] calculateShotVector(double robotHeading){
//        double g = 32.174*12;
//        double x = robotGoalVector.getMagnitude()-ShooterConstants.PASS_THROUGH_POINT_RADIUS;
//        double y = SCORE_HEIGHT;
//        double a = ShooterConstants.SCORE_ANGLE;
//
//
//        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x -Math.tan(a)), ShooterConstants.MAX_HOOD_ANGLE,
//                ShooterConstants.MIN_HOOD_ANGLE);
//
//        double flywheelSpeed = Math.sqrt(g * x * x / ( 2 * Math.pow( Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) -y )));
//
//        double coordinateTheta = robotVelocity.getTheta()-robotGoalVector.getTheta();
//
//        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        double vz = flywheelSpeed*Math.sin(hoodAngle);
//        double time = x/ (flywheelSpeed*Math.cos(hoodAngle));
//        double ivr = x / time+ parallelComponent;
//        double nvr = Math.sqrt(ivr*ivr+perpendicularComponent*perpendicularComponent);
//        double ndr =  nvr*time;
//
//        hoodAngle = MathFunctions.clamp(Math.atan(vz/nvr), ShooterConstants.MAX_HOOD_ANGLE,ShooterConstants.MIN_HOOD_ANGLE);
//        flywheelSpeed = Math.sqrt(g*ndr*ndr/(2*Math.pow(Math.cos(hoodAngle),2)*(ndr*Math.tan(hoodAngle)-y)));
//
//        double turretVelCompOffset = Math.atan(perpendicularComponent/ivr);
//        double turretAngle = Math.toDegrees(robotHeading-robotGoalVector.getTheta()+turretVelCompOffset);
//
//        turretAngle = Math.toDegrees(MathFunctions.normalizeAngle(Math.toRadians(turretAngle)));
//
//
//        return new double[]{
//                turretAngle,
//                getFlywheelTicksFromVelocity(flywheelSpeed),
//                hoodAngle
//        };
//    }
//    public static double[] getShootingFar(double distance) {
//        // Constants
//        double G = 386.088;
//        double SCORE_HEIGHT = ShooterConstants.SCORE_HEIGHT_FAR;
//        double PASS_THROUGH_RADIUS = ShooterConstants.PASS_THROUGH_POINT_RADIUS_FAR;
//
//        // Fixed Hood Angle (40 degrees from vertical)
//        double fixedVertAngleDeg = 44.0;
//        double fixedVertAngleRad = Math.toRadians(fixedVertAngleDeg);
//
//        // 1. Calculate target coordinates
//        double x = distance - PASS_THROUGH_RADIUS;
//        double y = SCORE_HEIGHT;
//
//        // 2. Convert Vertical Hood Angle to Horizontal Physics Angle
//        // If hood is 40 degrees from vertical, it is 50 degrees from horizontal
//        double finalHoriz = (Math.PI / 2) - fixedVertAngleRad;
//
//        // 3. Calculate Velocity needed to hit (x, y) at exactly 50 degrees horizontal
//        // Formula: v = sqrt( (g * x^2) / (2 * cos(theta)^2 * (x * tan(theta) - y)) )
//        double cosTheta = Math.cos(finalHoriz);
//        double tanTheta = Math.tan(finalHoriz);
//        double denom = (2 * Math.pow(cosTheta, 2) * (x * tanTheta - y));
//
//        // Check for physical impossibility (if 40 deg is too shallow to reach height y)
//        if (denom <= 0) {
//            return new double[] {0.0, fixedVertAngleDeg};
//        }
//
//        double velocity = Math.sqrt((G * Math.pow(x, 2)) / denom);
//
//        return new double[] {velocity, fixedVertAngleDeg};
//    }
    public static double getFlywheelTicksFromVelocity(double v){
        return com.pedropathing.math.MathFunctions.clamp(7.5421*v-486.76344, 0,Constants.LAUNCHER_MAX_VELOCITY);
    }
    public static double getFlywheelVelFromTicks(double v){ return com.pedropathing.math.MathFunctions.clamp((0.108461*v+71.41595), Constants.LAUNCHER_MIN_VELOCITY,Constants.LAUNCHER_MAX_VELOCITY); }
    public static double[] getShootingParams(double distance) {
        // Constants from your setup
        double G = 386.088;
        double SCORE_HEIGHT = ShooterConstants.SCORE_HEIGHT;
        double SCORE_ANGLE_RAD = ShooterConstants.SCORE_ANGLE;
        double PASS_THROUGH_RADIUS = ShooterConstants.PASS_THROUGH_POINT_RADIUS;

        // Mechanical Limits (Vertical Reference: 0 is straight up)
        double MIN_VERT_LIMIT = Math.toRadians(18);
        double MAX_VERT_LIMIT = Math.toRadians(44);

        // 1. Calculate target coordinates
        double x = distance - PASS_THROUGH_RADIUS;
        double y = SCORE_HEIGHT;

        // 2. Solve for ideal horizontal launch angle to hit -30 degree entry
        double targetHorizTheta = Math.atan((2 * y / x) - Math.tan(SCORE_ANGLE_RAD));

        // 3. Convert to Vertical Reference (Hood Angle)
        double targetVertTheta = (Math.PI / 2) - targetHorizTheta;

        // 4. Clamp to Mechanical Limits (Forces hood to stay <= 44 degrees)
        double clampedVert = Math.max(MIN_VERT_LIMIT, Math.min(MAX_VERT_LIMIT, targetVertTheta));

        // 5. Convert back to Horizontal for Velocity calculation
        double finalHoriz = (Math.PI / 2) - clampedVert;

        // 6. Calculate Velocity for the clamped angle
        // Using the kinematic trajectory equation solved for v
        double cosTheta = Math.cos(finalHoriz);
        double denom = (2 * Math.pow(cosTheta, 2) * (x * Math.tan(finalHoriz) - y));

        // Check for physical impossibility (if the target is higher than the peak)
        if (denom <= 0) {
            return new double[] {0.0, 0.0};
        }

        double velocity = Math.sqrt((G * Math.pow(x, 2)) / denom);
        double hoodAngleDeg = Math.toDegrees(clampedVert);

        return new double[] {velocity, hoodAngleDeg};
    }
    /**
     * Calculates the optimal vertical hood angle in degrees.
     * @param distance Distance from Robot to Goal (inches)
     * @param velocity Actual Flywheel Speed (inches/second)
     * @return Optimal vertical angle in degrees, or -1.0 if physically impossible.
     */
    public static double getHoodAngle(double distance, double velocity) {
        double G = 386.088;
        double x = distance - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;

        // Quadratic terms for tan(theta)
        double a = (G * Math.pow(x, 2)) / (2 * Math.pow(velocity, 2));
        double b = -x;
        double c = y + a;

        double discriminant = Math.pow(b, 2) - 4 * a * c;

        // If discriminant is negative, the ball can't reach the height at this speed
        if (discriminant < 0) return -1.0;

        // Calculate the two possible launch trajectories (High and Low)
        double tanTheta1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double tanTheta2 = (-b - Math.sqrt(discriminant)) / (2 * a);
        double[] options = {tanTheta1, tanTheta2};

        double bestVertAngle = -1.0;
        double minEntryError = Double.MAX_VALUE;
        double closestBoundAngle = -1.0;

        for (double tanT : options) {
            double hTheta = Math.atan(tanT);
            double vTheta = Math.toDegrees((Math.PI / 2) - hTheta); // Convert to Vertical Degrees

            // Calculate entry angle for this trajectory
            double tanEntry = tanT - (G * x) / (Math.pow(velocity, 2) * Math.pow(Math.cos(hTheta), 2));
            double entryAngle = Math.atan(tanEntry);
            double error = Math.abs(entryAngle - ShooterConstants.SCORE_ANGLE);

            // Check if within mechanical limits (18 to 44 degrees)
            if (vTheta >= 18.0 && vTheta <= 44.0) {
                if (error < minEntryError) {
                    minEntryError = error;
                    bestVertAngle = vTheta;
                }
            }

            // Track the "mathematically best" angle to clamp if no valid angle is found
            if (closestBoundAngle == -1.0 || error < minEntryError) {
                closestBoundAngle = Math.max(18.0, Math.min(44.0, vTheta));
            }
        }

        // Return the best valid angle, or the clamped version of the best trajectory
        return (bestVertAngle != -1.0) ? bestVertAngle : closestBoundAngle;
    }


}
