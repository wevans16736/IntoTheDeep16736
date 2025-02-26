package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.MecanumDrive.FollowTrajectoryAction;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.MotorConstants;
import org.firstinspires.ftc.teamcode.secondrobot.DetectBlockActions;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;
import org.opencv.core.Point;

import java.util.Objects;

public class RobotSensor {

    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(0));
    Telemetry telemetry; PinpointDrive drive; FtcDashboard dashboard; Telemetry dashboardTelemetry; DistanceSensor distance;
    DetectBlockActions vision; HardwareMap hardwareMap;
    DcMotorEx FR; DcMotorEx FL; DcMotorEx RR; DcMotorEx RL;

    public RobotSensor(Telemetry telemetry, PinpointDrive drive, DetectBlockActions vision){
        this.telemetry = telemetry;
        this.drive = drive;
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.vision = vision;
    }

    public class RobotSensorAction implements Action{
        String Location; double x; double y; double heading;
        public RobotSensorAction(String Location, double x, double y, double heading){
            this.Location = Location;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double headingError = drive.getLastPinpointPose().heading.toDouble() - heading;
            double xError = drive.getLastPinpointPose().position.x - x;
            double yError = drive.getLastPinpointPose().position.y - y;

            telemetry.addData(Location+"heading",headingError);
            telemetry.addData(Location+"xError",xError);
            telemetry.addData(Location+"yError",yError);
            telemetry.addLine("\n");

            dashboardTelemetry.addData(Location+": heading", headingError);
            dashboardTelemetry.addData(Location+": xError",xError);
            dashboardTelemetry.addData(Location+": yError",yError);
            dashboardTelemetry.addLine("\n");

            telemetry.update();
            dashboardTelemetry.update();
            return false;
        }
    }
    public class VisionOn implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            telemetry.addLine("opening");
            telemetry.update();
            vision.activate();
            telemetry.addLine("opened");
            telemetry.update();
            return false;
        }
    }
    public class VisionScan implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Point pixelLoc = vision.getCenter();
            Point point = vision.pixelToPosition(pixelLoc);
            GlobalVariables.X = point.x*(-1) + drive.getLastPinpointPose().position.x;
            GlobalVariables.Y = vision.pixelToPosition(vision.getCenter()).y;
            GlobalVariables.Roll = vision.getAngle();


            if(!Objects.equals(pixelLoc, new Point(0, 0))){
                vision.deactivate();
                return false;
            } else
                return true;
        }
    }
    public Action visionOn(){return new VisionOn();}
    public Action visionScan(){ return new VisionScan();}
    public Action robotSensorAction(String Location, double x, double y, double heading){return new RobotSensorAction(Location, x, y, heading);}

}
