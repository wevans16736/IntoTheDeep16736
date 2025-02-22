package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Decapricated;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

public class Trajectory {
    VerticalSlideRR verticalSlideRR;
    VerticalWristRR verticalWristRR;
    VerticalGrabberRR verticalGrabberRR;
    VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR;
    HorizontalRollRR horizontalRollRR;
    HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR;
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    RobotSensor robotSensor;
    VelConstraint butterSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(ConfigurationSecondRobot.driveVelocity),
            new AngularVelConstraint(Math.PI * 8)
    ));
    double hangX = 0;
    double hangY = -33;
    double firstButterX = 47;
    double firstButterY = -36;
    double secondButterX = 58.75;
    double secondButterY = -43;
    double thirdButterX = 58;
    double thirdButterY = -25.75;
    double humanX = 42;
    double humanY = -50.125;
    int counter = 0;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR, RobotSensor robotSensor) {
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

    public Action getHang() {
        if (counter == 0) {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .afterTime(0, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(0, hangY - 3, Math.toRadians(90)), Math.toRadians(-90))
                    .stopAndAdd(robotSensor.robotSensorAction("first_hang", 0, hangY - 3, 90));
            currentTrajectory = Hang.endTrajectory().fresh();
            counter++;
            hangX = -11;
            return Hang.build();
        }
        if (counter == 1) {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseWideTime / 1000)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90), butterSpeed)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(hangX, hangY - 3, Math.toRadians(90)), Math.toRadians(-90))
                    .afterTime(1, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .afterTime(1, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(humanX, humanY+2.25, Math.toRadians(-90.00001)), Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.0001)), Math.toRadians(0))
                    .stopAndAdd(robotSensor.robotSensorAction("hang " + counter, humanX, humanY, -90));
            currentTrajectory = Hang.endTrajectory().fresh();
            counter++;
            hangX += 3;
            return Hang.build();
        } else {
            TrajectoryActionBuilder Hang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseWideTime / 1000)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY + 2, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(hangX, hangY - 3, Math.toRadians(90)), Math.toRadians(-90))
                    .afterTime(1, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .afterTime(1, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(humanX, humanY+2.25, Math.toRadians(-90.00001)), Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(humanX, humanY - .35, Math.toRadians(-90.00001)), Math.toRadians(-90))
                    .stopAndAdd(robotSensor.robotSensorAction("hang " + counter, humanX, humanY, -90));
            currentTrajectory = Hang.endTrajectory().fresh();
            hangX += 3;
            return Hang.build();
        }
    }

    public Action getFirstButter() {
        TrajectoryActionBuilder FirstButter = currentTrajectory
                .afterTime(.5, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .afterTime(.5, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
//                .afterDisp(30, horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .afterDisp(30, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .afterDisp(30, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                //first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.01)), Math.toRadians(90), butterSpeed)
                .stopAndAdd(robotSensor.robotSensorAction("first_butter", firstButterX, firstButterY, -90));
        currentTrajectory = FirstButter.endTrajectory().fresh();
        return FirstButter.build();
    }

    public Action getSecondButter() {
        TrajectoryActionBuilder SecondButter = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                .waitSeconds(.1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0), butterSpeed)
                .stopAndAdd(robotSensor.robotSensorAction("second_butter", secondButterX, secondButterY, -90));
        currentTrajectory = SecondButter.endTrajectory().fresh();
        return SecondButter.build();
    }

    public Action getThirdButter() {
        TrajectoryActionBuilder ThirdButter = currentTrajectory
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY - 4.5, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                .afterTime(.25, horizontalSlideRR.horizontalSlideActions(350))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(90), butterSpeed)
                .stopAndAdd(robotSensor.robotSensorAction("third_butter", thirdButterX, thirdButterY, 180))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                //go to human pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56, -60, Math.toRadians(-.000001)), Math.toRadians(0), butterSpeed)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(robotSensor.robotSensorAction("side_wall", 56, -60, 0));
        currentTrajectory = ThirdButter.endTrajectory().fresh();
        return ThirdButter.build();
    }

    public Action getTransfer(boolean extend) {
        if (!extend) {
            return currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .build();
        } else {
            return currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
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
                .splineToLinearHeading(new Pose2d(0, hangY - 5, Math.toRadians(90)), Math.toRadians(-90))
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
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(.5)
                //go to human pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(55, -65, Math.toRadians(-.000001)), Math.toRadians(0))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3, hangY - 5, Math.toRadians(90)), Math.toRadians(-90))
                //go back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY - 5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY - 5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY - 5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .build();
    }

    public Action testTransfer(boolean extend) {
        if (extend) {
            return currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .waitSeconds(1)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .build();
        } else {
            return currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .waitSeconds(1)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .build();
        }
    }
}
