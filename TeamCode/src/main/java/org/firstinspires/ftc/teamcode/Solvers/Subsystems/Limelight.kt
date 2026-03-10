package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import java.util.Objects;

public class Limelight extends SubsystemBase {
    Robot robot = Robot.getInstance();
    Limelight3A limelight3A;
    Follower follower;
    public Double lastDistance = 220.0;
    public static Boolean hasResult = false;


    public Limelight(){

    }
    public static Pose toPedro(Pose pose) {
        double x = (pose.getX() * 39.37) * -1;
        double y = pose.getY() * 39.37;
        double heading = pose.getHeading();
        return new Pose(x, y, heading);
    }
    public LLResult grabResultData() {
        return limelight3A.getLatestResult();
    }
    public Pose megaTag() {
        LLResult lR = grabResultData();
        if (!lR.isValid()) {

            hasResult = false;
            return null;
        }

        Pose3D botpose_mt2 = lR.getBotpose_MT2();
        if (botpose_mt2 == null) {

            hasResult = false;
            return null;
        }


        Position position = botpose_mt2.getPosition();
        Pose pose = new Pose(
                position.y,
                position.x*-1,
                robot.CurrentPose.getHeading()
        );
        hasResult = true;
        return new Pose(pose.getX()* 39.37,pose.getY()* 39.37,lR.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
    }

    public double getTx(){
        LLResult lR = grabResultData();
        if (!lR.isValid()) {
            return 0.0;
        };
        return lR.getTx();
    }
    public boolean withingDead(){
        double tx = getTx();
        robot.telemetryData.addData("tx",tx);
        return  tx<2;
    }
    public double getDistance() {
        Pose robotP = robot.CurrentPose;
        if (robotP == null) return lastDistance; // or lastDistance

        Pose goal = (Objects.equals(ALLIANCE_COLOR, "BLUE"))
                ? blueGoalPose
                : redGoalPose;

        double dx = goal.getX() - robotP.getX();
        double dy = goal.getY() - robotP.getY();
        double distance = Math.hypot(dx, dy);

        lastDistance = distance;
        return distance;
    }
}
