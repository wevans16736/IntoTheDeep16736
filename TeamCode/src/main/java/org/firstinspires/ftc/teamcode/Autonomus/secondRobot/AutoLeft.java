package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.DetectBlockActions;
@Autonomous(name = "AutoLeft")
public class AutoLeft extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            //set up Pinpoint and Pose2d class
            Pose2d pose;
            PinpointDrive drive;
            boolean side = true;

            //all of these class is under Configuration.secondRobot
            VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
            VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
            VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

            if (!side) {
                pose = new Pose2d(9, -64, Math.toRadians(90));
                drive = new PinpointDrive(hardwareMap, pose);
            } else {
                pose = new Pose2d(-15, -64, Math.toRadians(180));
                drive = new PinpointDrive(hardwareMap, pose);
            }

            HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
            HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
            HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
            HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
            VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

            DetectBlockActions vision = new DetectBlockActions(hardwareMap);

            TrajectoryLeft trajectory = new TrajectoryLeft(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                    verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);

            //build everything
            Action Basket1 = trajectory.getBasket();
            Action Butter1 = trajectory.getButter();
            Action Basket2 = trajectory.getBasket();
            Action Attachment1 = trajectory.getAttachment();
            Action Butter2 = trajectory.getButter();
            Action Basket3 = trajectory.getBasket();
            Action Attachment2 = trajectory.getAttachment();
            Action Butter3 = trajectory.getButter();
            Action Basket4 = trajectory.getBasket();
            Action Park1 = trajectory.getPark();
            Action Basket5 = trajectory.getBasket();
            Action Attachment3 = trajectory.getAttachment();
            Action Attachment4 = trajectory.getAttachment();
            Action Park2 = trajectory.getPark();
            //wait for the start button to be press
            waitForStart();
            //if the stop button press then stop the robot
            if (isStopRequested()) return;

            Actions.runBlocking(new SequentialAction(
                    Basket1,
                    new SleepAction(1.8),
                    Park1,
                    new ParallelAction(
                            Basket5,
                            Attachment4
                    ),
                    Butter1,
                    new ParallelAction(
                            Basket2,
                            Attachment1
                    ),
                    Butter2,
                    new ParallelAction(
                            Basket3,
                            Attachment2
                    ),
                    Butter3,
                    new ParallelAction(
                            Basket4,
                            Attachment3
                    ),
                    Park2
                    ));
            GlobalVariables.autoStarted = true;
            GlobalVariables.currentPose = drive.getLastPinpointPose();
        }
}
