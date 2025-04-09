package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.RR.GlobalVariables;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

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

//            DetectBlockActions vision = new DetectBlockActions(hardwareMap);

            TrajectoryLeft trajectory = new TrajectoryLeft(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                    verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);

            //build everything
           Action basket1 = trajectory.getBasket();
           Action butter1 = trajectory.getButter();
           Action basket2 = trajectory.getBasket();
           Action attachment1 = trajectory.getAttachment();
           Action butter2 = trajectory.getButter();
           Action basket3 = trajectory.getBasket();
           Action attachment2 = trajectory.getAttachment();
           Action butter3 = trajectory.getButter();
           Action basket4 = trajectory.getBasket();
           Action attachment3 = trajectory.getAttachment();
           Action guess1 = trajectory.getGuess();
           Action basket5 = trajectory.getBasket();
           Action attachment4 = trajectory.getAttachment();
           Action guess2 = trajectory.getGuess();

           boolean attempt = true; int distance;

           while(!gamepad1.cross || opModeIsActive()){
               telemetry.addData("attempt", attempt);
               telemetry.update();
               if(gamepad1.square){
                   attempt = !attempt;
               }
           }
           telemetry.addLine("ready");
           telemetry.update();
            //wait for the start button to be press
            waitForStart();
            //if the stop button press then stop the robot
            if (isStopRequested()) return;
            if(attempt) {
                Actions.runBlocking(new SequentialAction(
                        basket1,
                        butter1,
                        new ParallelAction(
                                basket2,
                                attachment1
                        ),
                        butter2,
                        new ParallelAction(
                                basket3,
                                attachment2
                        ),
                        butter3,
                        new ParallelAction(
                                basket4,
                                attachment3
                        ),
                        guess1,
                        new ParallelAction(
                                basket5,
                                attachment4
                        ),
                        guess2
                ));
                GlobalVariables.autoStarted = true;
                GlobalVariables.currentPose = drive.getLastPinpointPose();
            } else {
                Actions.runBlocking(new SequentialAction(
                        basket1,
                        butter1,
                        new ParallelAction(
                                basket2,
                                attachment1
                        ),
                        butter2,
                        new ParallelAction(
                                basket3,
                                attachment2
                        ),
                        butter3,
                        new ParallelAction(
                                basket4,
                                attachment3
                        ),
                        guess2
                ));
            }
        }
}
