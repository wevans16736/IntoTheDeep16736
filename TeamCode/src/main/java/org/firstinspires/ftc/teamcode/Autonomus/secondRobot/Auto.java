package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
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

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabberRR = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap);

        //set up a trajectory class
        Trajectory trajectory = new Trajectory();
        trajectory.setTrajectory(drive, pose, verticalSlideRR, verticalWristRR, verticalGrabberRR, horizontalSlideRR,
                horizontalRollRR, horizontalGrabberRR, horizontalWristRR);
        trajectory.setStartTrajectory();

        //todo ask the driver which trajectory to use

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

//        run hanging trajectory
        Actions.runBlocking(new SequentialAction(
                        trajectory.getHangTrajectory().build(),
                trajectory.getButterPickUpTrajectory().build(),
                new ParallelAction(
                        trajectory.getButterPickUpAttachment().build(),
                        trajectory.getSecondButterPickUpTrajectory().build()
                ),
                trajectory.getButterPickUpAttachment().build(),
                trajectory.getPostHangTrajectory().build()

                ));
    }
}
