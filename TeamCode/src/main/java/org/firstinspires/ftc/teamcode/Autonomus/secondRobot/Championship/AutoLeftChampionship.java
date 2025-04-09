package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Championship;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.LimeSweet;

@Autonomous(name = "AutoLeftChampionship", group = "auto")
public class AutoLeftChampionship extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose = new Pose2d(-39.5,-62.25,Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose);
        LimeSweet lime = new LimeSweet(hardwareMap, telemetry, 0);

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);


        TrajectoryLeftChampionship trajectory = new TrajectoryLeftChampionship(drive, pose,lime, horizontalGrabber, horizontalRoll,
                horizontalSlide, horizontalWrist, verticalGrabber, verticalHanger, verticalSlide, verticalWrist);

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                trajectory.getBasket(),
                trajectory.getButter(),
                new ParallelAction(
                        trajectory.getBasket(),
                        trajectory.getTransfer()
                ),
                trajectory.getButter(),
                new ParallelAction(
                        trajectory.getBasket(),
                        trajectory.getTransfer()
                ),
                trajectory.getButter(),
                new ParallelAction(
                        trajectory.getBasket(),
                        trajectory.getTransfer()
                ),
                trajectory.getSub(),
                new SleepAction(2000)
        ));
//        trajectory.getButterPose();
//        Actions.runBlocking(new SequentialAction(
//                trajectory.getSubButter(),
//                new ParallelAction(
//                        trajectory.getBasket(),
//                        trajectory.getTransfer()
//                )
//        ));
    }
}
