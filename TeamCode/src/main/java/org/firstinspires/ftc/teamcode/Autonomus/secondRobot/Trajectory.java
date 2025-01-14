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
                .splineTo(new Vector2d(0.0, 20.0),Math.toRadians(90.0))
                ;
        currentTrajectory = hangTrajectory.endTrajectory().fresh();
        return hangTrajectory.build();
    }
    public Action getButterLocation(){
        TrajectoryActionBuilder butterLocation;
        butterLocation = currentTrajectory
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(22.0, 10.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .splineToLinearHeading(new Pose2d(40.0, 18.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                //at the first butter
                .waitSeconds(1)
                .splineTo(new Vector2d(50.0, 18.0), Math.toRadians(0))
                //at the second butter
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(40.0, 40.0, Math.toRadians(180.0)), Math.toRadians(-90.0))
                //at the third butter
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(40.0, 14.0, Math.toRadians(-90.0)), Math.toRadians(180.0))
        ;
        currentTrajectory = butterLocation.endTrajectory().fresh();
        return butterLocation.build();
    }

    public Action getPostHangTrajectory(){
        TrajectoryActionBuilder postHangTrajectory;
        postHangTrajectory = currentTrajectory

        ;
        currentTrajectory = postHangTrajectory.endTrajectory().fresh();
        return postHangTrajectory.build();
    }
}
