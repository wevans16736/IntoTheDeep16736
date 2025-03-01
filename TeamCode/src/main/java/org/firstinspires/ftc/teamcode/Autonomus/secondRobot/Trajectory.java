package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

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
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

public class Trajectory{
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    RobotSensor robotSensor;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR){
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

        currentTrajectory = drive.actionBuilder(pose);
    }
    double hangX = -1; double hangY = -34.5; int hang = 0; int attachment = 0;
    double firstButterX = 49; double firstButterY = -36.25; double butterCounter = 0;
    double secondButterX = 58; double secondButterY =  -47.5;
    double thirdButterX = 61; double thirdButterY = -28.5;
    double humanX = 43; double humanY = -52.5; int human = 0;

    public Action getHang(){
        if(hang == 0){
            TrajectoryActionBuilder firstHang = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalOpenTime/1000);
            hangX = 1;
            hang++;
            currentTrajectory = firstHang.endTrajectory().fresh();
            return firstHang.build();
        } if(hang == 1){
            TrajectoryActionBuilder secondHang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000 + .1)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(200))
                    .splineToLinearHeading(new Pose2d(hangX, hangY + .5, Math.toRadians(90)), Math.toRadians(115))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
//                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristUp))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000);
            hangX += 1.5;
            hang++;
            currentTrajectory = secondHang.endTrajectory().fresh();
            return secondHang.build();
        } else {
            TrajectoryActionBuilder thirdHang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalClose/1000 + .3)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristUp))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .afterTime(1, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .strafeToLinearHeading(new Vector2d( humanX - 15, humanY), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(hangX, hangY), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen));
            hangX += 1.5;
            currentTrajectory = thirdHang.endTrajectory().fresh();
            return thirdHang.build();
        }
    }
    public Action getButter(){
        if(butterCounter == 0){
            TrajectoryActionBuilder firstButter = currentTrajectory
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.0001)), Math.toRadians(90));
            butterCounter++;
            currentTrajectory = firstButter.endTrajectory().fresh();
            return firstButter.build();
        } if(butterCounter == 1){
            TrajectoryActionBuilder secondButter = currentTrajectory
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0));
            butterCounter++;
            currentTrajectory = secondButter.endTrajectory().fresh();
            return secondButter.build();
        } else {
            TrajectoryActionBuilder thirdButter = currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(350))
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(0))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(55.5, -61.25, Math.toRadians(-.00000000001)), Math.toRadians(0))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000);
            currentTrajectory = thirdButter.endTrajectory().fresh();
            return thirdButter.build();
        }
    }
    public Action getAttachment(){
        if(attachment == 0){
            TrajectoryActionBuilder firstAttachment = currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalOpenTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000);
            attachment++;
            currentTrajectory = firstAttachment.endTrajectory().fresh();
            return  firstAttachment.build();
        } else {
            TrajectoryActionBuilder secondAttachment = currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalOpenTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000);
            currentTrajectory = secondAttachment.endTrajectory().fresh();
            return secondAttachment.build();
        }
    }
    public Action getHuman(){
        TrajectoryActionBuilder Human = currentTrajectory
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristUp))
                .afterTime(1, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-89.9999999999)), Math.toRadians(-90));
        currentTrajectory = Human.endTrajectory().fresh();
        return Human.build();
    }
}