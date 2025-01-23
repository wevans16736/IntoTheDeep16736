package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.*;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
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
    double hangX = 0;
    double hangY = -34;
    double firstButterX = 48.5;
    double firstButterY = -40;
    double secondButterX = 59;
    double secondButterY = -40;
    double thirdButterX = 60;
    double thirdButterY = -27;
    double humanX = 40;
    double humanY = -60;
    int counter = 0;
    public Action getHang(){
        if(counter == 0) {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .afterTime(0, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(0, hangY - 5, Math.toRadians(90)), Math.toRadians(-90));
            currentTrajectory.endTrajectory().fresh();
            counter++;
            hangX = -5;
            return Hang.build();
        } if(counter == 1){
            TrajectoryActionBuilder Hang = currentTrajectory
                    .waitSeconds(.2)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(hangX, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90));
            currentTrajectory.endTrajectory().fresh();
            counter++;
            hangX += 2;
            return Hang.build();
        } else{
            TrajectoryActionBuilder Hang = currentTrajectory
                    .waitSeconds(.2)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(hangX, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90));
            currentTrajectory.endTrajectory().fresh();
            hangX +=2;
            return Hang.build();
        }
    }
    public Action getFirstButter(){
        TrajectoryActionBuilder FirstButter = currentTrajectory
                .afterDisp(20, horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .afterDisp(20, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .afterDisp(20, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                //first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.01)), Math.toRadians(90));
        currentTrajectory.endTrajectory().fresh();
        return FirstButter.build();
    }
    public Action getSecondButter(){
        TrajectoryActionBuilder SecondButter = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0));
        currentTrajectory = SecondButter.endTrajectory().fresh();
        return SecondButter.build();
    }
    public Action getThirdButter(){
        TrajectoryActionBuilder ThirdButter = currentTrajectory
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY , Math.toRadians(180)), Math.toRadians(90))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                //go to human pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-.000001)), Math.toRadians(0))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen));
        currentTrajectory.endTrajectory().fresh();
        return ThirdButter.build();
    }
    public Action getTransfer(boolean extend){
        if(!extend) {
            return currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .build();
        } else{
            return currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                   stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .build();
        }
    }
    public Action getAlltrajectory() {
        return currentTrajectory
                //hang
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(.5)
                //back up a little
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.01)), Math.toRadians(90))
                .waitSeconds(.5)
                //second butter
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(.5)
                //third butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY , Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(.5)
                //go to human pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-.000001)), Math.toRadians(0))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .build();
    }
    public Action testTransfer(boolean extend){
        if(extend){
            return currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .waitSeconds(1)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .build();
        }
        else{
            return currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .waitSeconds(1)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .build();
        }
    }
}
