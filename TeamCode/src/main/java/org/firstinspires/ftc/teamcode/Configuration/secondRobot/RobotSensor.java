package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class RobotSensor {

    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(0));
    Telemetry telemetry; PinpointDrive drive; FtcDashboard dashboard; Telemetry dashboardTelemetry; DistanceSensor distance;
    HardwareMap hardwareMap;
    DcMotorEx FR; DcMotorEx FL; DcMotorEx RR; DcMotorEx RL;

    public RobotSensor(Telemetry telemetry, PinpointDrive drive){
        this.telemetry = telemetry;
        this.drive = drive;
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
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
    public Action robotSensorAction(String Location, double x, double y, double heading){return new RobotSensorAction(Location, x, y, heading);}

}
