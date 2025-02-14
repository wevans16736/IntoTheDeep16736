package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodAccess;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Trajectory{
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    RobotSensor robotSensor;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR, RobotSensor robotSensor){
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
        this.robotSensor = robotSensor;

        currentTrajectory = drive.actionBuilder(pose);
    }
    double hangX = 0; double hangY = -30; boolean firstHang = true;
    double firstButterX = 35; double firstButterY = -45; double butterCounter = 0;
    double secondButterX = 45; double secondButterY = firstButterY;
    double thirdButterX = 55; double thirdButterY = firstButterY;
    double humanX = 61; double humanY = -58;
    public Action getHang(){
        if(firstHang) {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY -3 , Math.toRadians(90)), Math.toRadians(-90));
            currentTrajectory = Hang.endTrajectory().fresh();
            firstHang = false;
            hangX = -10;
            return Hang.build();
        } else {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .setTangent(180)
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY - 3, Math.toRadians(90)), Math.toRadians(-90));
            currentTrajectory = Hang.endTrajectory().fresh();
            hangX += 2;
            return Hang.build();
        }
    }
    public Action getHuman(){
        TrajectoryActionBuilder Human = currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(0)), Math.toRadians(0));
        currentTrajectory = Human.endTrajectory().fresh();
        return Human.build();
    }
    public Action getButter(){
        if(butterCounter == 0) {
            TrajectoryActionBuilder FirstButter = currentTrajectory
                    .afterDisp(5, horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .afterDisp(5, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .afterDisp(5, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-135)), Math.toRadians(0));
            currentTrajectory = FirstButter.endTrajectory().fresh();
            butterCounter++;
            return FirstButter.build();
        } if(butterCounter == 1){
            TrajectoryActionBuilder SecondButter = currentTrajectory
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-135)), Math.toRadians(0));
            currentTrajectory = SecondButter.endTrajectory().fresh();
            butterCounter++;
            return SecondButter.build();
        } else {
            TrajectoryActionBuilder ThirdButter = currentTrajectory
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(-135)), Math.toRadians(0));
            currentTrajectory = ThirdButter.endTrajectory().fresh();
            return ThirdButter.build();
        }
    }
    public Action getButterAttachment(){
        TrajectoryActionBuilder ButterAttachment = currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .turn(Math.toRadians(135))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000);
        currentTrajectory = ButterAttachment.endTrajectory().fresh();
        return ButterAttachment.build();
    }
    public Action getScanButter(){
        TrajectoryActionBuilder ScanButter = currentTrajectory
                .stopAndAdd(robotSensor.distanceSensorAction())
                .strafeTo(new Vector2d(humanX, humanY +2));
        currentTrajectory = ScanButter.endTrajectory().fresh();
        return ScanButter.build();
    }

    public void transferPose(){
//        GlobalVariables.currentPose = cur
    }
}