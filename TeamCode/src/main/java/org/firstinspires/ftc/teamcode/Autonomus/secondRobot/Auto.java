package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name="Auto", group = "SecondRobot")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose = new Pose2d(0,0,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose);

        //set up a trajectory class
        Trajectory trajectory = new Trajectory(drive, pose, hardwareMap);

        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                trajectory.initialize.build()
                ));

        //todo ask the driver which trajectory to use

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        //run hanging trajectory
        Actions.runBlocking(new SequentialAction(
                        trajectory.HangTrajectory.build()
                ));

        //setup current trajectory as hang trajectory
        trajectory.setCurrentPose(trajectory.HangTrajectory);

        //move the robot to where the butter is located
        Actions.runBlocking(new SequentialAction(
                trajectory.runButterLocation()));

        //update the current pose in trajectory
        trajectory.setCurrentPose(trajectory.ButterPickupTrajectory);

        //pick all the butter and drop the behind
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        trajectory.runButterPickUp(),
                        new SequentialAction(
                                trajectory.ButterPickUpAttachment.build(),
                                trajectory.ButterPickUpAttachment.build()
                        )
                )
        ));

        //update the current pose in trajectory
        trajectory.setCurrentPose(trajectory.ButterPickUp);
    }
}
