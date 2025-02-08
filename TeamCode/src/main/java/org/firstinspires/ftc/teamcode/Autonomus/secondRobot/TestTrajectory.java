package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class TestTrajectory {
        VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR;
        VerticalHangerRR verticalHangerRR; HorizontalSlideRR horizontalSlideRR;
        HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
        HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose;
        TrajectoryActionBuilder currentTrajectory; RobotSensor robotSensor;
        public TestTrajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                              VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                              VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR,
                              HorizontalRollRR horizontalRollRR, HorizontalGrabberRR horizontalGrabberRR,
                              HorizontalWristRR horizontalWristRR, RobotSensor robotSensor) {
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
    VelConstraint basketSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25),
            new AngularVelConstraint(Math.PI)
    ));

        public Action getTest(){
            TrajectoryActionBuilder Test = currentTrajectory
//                    .splineToLinearHeading(new Pose2d(-47.25/2, 15, Math.toRadians(180)), Math.toRadians(180), basketSpeed)
//                    .splineToLinearHeading(new Pose2d(-47.25, 0, Math.toRadians(-90)), Math.toRadians(-90), basketSpeed)
                    .strafeTo(new Vector2d(0, 50));
//                    .strafeTo(new Vector2d(0,0));
            currentTrajectory = Test.endTrajectory().fresh();
            return Test.build();
        }
        public Action getTest2(){
            TrajectoryActionBuilder Test2 = currentTrajectory
//                    .stopAndAdd(robotSensor.robotSensorAction("First Position", 0, 50, 90))
                    .strafeTo(new Vector2d(0,0));
            currentTrajectory = Test2.endTrajectory().fresh();
            return Test2.build();
        }
        public TrajectoryActionBuilder getCurrentTrajectory(){
            return currentTrajectory;
        }
}
