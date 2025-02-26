package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.LimeLightTesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.List;

public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    Limelight3A lime; double x = 0.0; double y = 0.0; Telemetry telemetry;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR, Limelight3A lime, Telemetry telemetry){
        this.drive = drive;
        this.pose = pose;
        this.verticalSlideRR = verticalSlideRR;
        this.verticalWristRR = verticalWristRR;
        this.verticalGrabberRR = verticalGrabberRR;
        this.verticalHangerRR = verticalHangerRR;
        this.horizontalSlideRR = horizontalSlideRR;
        this.horizontalRollRR = horizontalRollRR;
        this.horizontalGrabberRR = horizontalGrabberRR;
        this.horizontalWristRR = horizontalWristRR;
        this.lime = lime;
        currentTrajectory = drive.actionBuilder(pose);
    }
    public Action getInital() {
        TrajectoryActionBuilder inital = currentTrajectory
                .splineToLinearHeading(new Pose2d(-15, 25, Math.toRadians(180)), Math.toRadians(180));
        currentTrajectory = inital.endTrajectory().fresh();
        return inital.build();
    }
    public void updateLime(){
//        x = 5;
//        y = 60;
//        //todo figure out how to get the butter pose
//        x = drive.getLastPinpointPose().position.x;
//        y = drive.getLastPinpointPose().position.y;
//        LLResult result = lime.getLatestResult();
//        if(result != null) {
//            // Access general information
//            Pose3D botpose = result.getBotpose();
//            double captureLatency = result.getCaptureLatency();
//            double targetingLatency = result.getTargetingLatency();
//            double parseLatency = result.getParseLatency();
//            telemetry.addData("LL Latency", captureLatency + targetingLatency);
//            telemetry.addData("Parse Latency", parseLatency);
//            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//
//            if (result.isValid()) {
//                double angle = 90 - result.getTy()- .5;
//                y  += Math.tan(angle)*4.4375;
//                x += Math.tan(result.getTx())*y;
//
//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("txnc", result.getTxNC());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("tync", result.getTyNC());
//                telemetry.addData("X", x);
//                telemetry.addData("Y", y);
//
//                telemetry.addData("Botpose", botpose.toString());
//            } else {
//                telemetry.addData("Limelight", "No data available");
//            }
//        }
    }
    public Action getFinal(){
        TrajectoryActionBuilder Final = drive.actionBuilder(new Pose2d(drive.getLastPinpointPose().position.x, drive.getLastPinpointPose().position.y, drive.getLastPinpointPose().heading.toDouble()))
                .splineToLinearHeading(new Pose2d(-60, 5, Math.toRadians(-90)), Math.toRadians(-90));
//        telemetry.addData("X", x);
//        telemetry.addData("Y", y);
        currentTrajectory = Final.endTrajectory().fresh();
        return Final.build();
    }
    public Action getButter(){
        updateLime();
        TrajectoryActionBuilder butter = currentTrajectory
                .strafeToConstantHeading(new Vector2d(x, y));
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        currentTrajectory = butter.endTrajectory().fresh();
        return butter.build();
    }
}
