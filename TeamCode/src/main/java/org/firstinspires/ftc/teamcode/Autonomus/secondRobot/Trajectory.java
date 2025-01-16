package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
    VelConstraint slowerVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(Math.PI*.75)
    ));

    public Action getAlltrajectory() {
        return currentTrajectory
                .splineTo(new Vector2d(9, -38), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(48, -40, Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(58, -40), Math.toRadians(0))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(45, -28, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(1)

                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-9, -40, Math.toRadians(90.0000001)), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(.5)
                .build();
    }
    public Action testTransfer(){
        return currentTrajectory
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .build();
    }
}
