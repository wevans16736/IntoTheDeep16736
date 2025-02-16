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

public class TrajectoryTest {
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

    public TrajectoryTest(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
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

    VelConstraint butterSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(50),
            new AngularVelConstraint(Math.PI)
    ));
    public Action getTest(){
        TrajectoryActionBuilder Test = currentTrajectory
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 0, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(180));

        currentTrajectory = Test.endTrajectory().fresh();
        return Test.build();
    }
}