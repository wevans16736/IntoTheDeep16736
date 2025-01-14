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
            new TranslationalVelConstraint(50),
            new AngularVelConstraint(Math.PI)
    ));

    public Action getHangTrajectory(){
        TrajectoryActionBuilder hangTrajectory;
        hangTrajectory = currentTrajectory

                .splineTo(new Vector2d(0, 20), Math.toRadians(90))

        ;
        currentTrajectory = hangTrajectory.endTrajectory().fresh();
        return hangTrajectory.build();
    }
    public Action getFirstButterTrajectory(){
        TrajectoryActionBuilder firstButterTrajectory;
        firstButterTrajectory = currentTrajectory
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .splineToLinearHeading(new Pose2d(40, 18, Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .splineTo(new Vector2d(50, 18), Math.toRadians(0))
        ;
        currentTrajectory = firstButterTrajectory.endTrajectory().fresh();
        return firstButterTrajectory.build();
    }
    public Action getSecondButterTrajectory(){
        TrajectoryActionBuilder secondButterTrajectory;
        secondButterTrajectory = currentTrajectory
                //todo
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)), Math.toRadians(-90))

        ;
        currentTrajectory = secondButterTrajectory.endTrajectory().fresh();
        return secondButterTrajectory.build();
    }
    public Action getThirdButterTrajectory(){
        TrajectoryActionBuilder thirdButterTrajectory;
        thirdButterTrajectory = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .splineToLinearHeading(new Pose2d(40, 14, Math.toRadians(-90)), Math.toRadians(180))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                ;
        currentTrajectory = thirdButterTrajectory.endTrajectory().fresh();
        return thirdButterTrajectory.build();
    }

    public Action getPostHangTrajectory(){
        TrajectoryActionBuilder postHangTrajectory;
        postHangTrajectory = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .splineToLinearHeading(new Pose2d(17, 10, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .splineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(3.0, 10.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .splineToLinearHeading(new Pose2d(20.0, 10.0, Math.toRadians(-90.0)), Math.toRadians(0.0));
        currentTrajectory = postHangTrajectory.endTrajectory().fresh();
        return postHangTrajectory.build();
    }
}
