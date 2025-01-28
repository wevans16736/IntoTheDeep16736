package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

//        DriverRequest driverRequest = new DriverRequest();
        boolean side = false; //determine which side
        boolean sideway = false; //determine roll if left choosen
        boolean attempt = true; //determine if we even want to try blind pick
        telemetry.clearAll();
        while(!gamepad1.right_bumper){
            telemetry.addLine("Choose which side? (circle)");
            telemetry.addData("Left Side ", side);
            telemetry.addData("Right Side", !side);
            if(gamepad1.circle){
                side = !side;
            }
            telemetry.update();
            sleep(500);
        }
        sleep(1000);
        telemetry.clearAll();
        if(side){
            while(!gamepad1.right_bumper) {
                telemetry.addLine("Sideway required? (circle)");
                telemetry.addData("Sideway: ", sideway);
                telemetry.addData("Attempt blind grab? (square)", attempt);
                if(gamepad1.circle){
                    sideway = !sideway;
                }
                if(gamepad1.square){
                    attempt = !attempt;
                }
                telemetry.update();
                sleep(500);
            }
        }
        if(!side){
            pose = new Pose2d(9,-64,Math.toRadians(90));
            drive = new PinpointDrive(hardwareMap, pose);
        }else{
            pose = new Pose2d(-15,-64,Math.toRadians(180));
            drive = new PinpointDrive(hardwareMap, pose);
        }

        Trajectory trajectory = new Trajectory(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);

        TrajectoryLeft trajectoryLeft = new TrajectoryLeft(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);


        Action getHang1Built = trajectory.getHang();
        Action getFirstButterBuilt = trajectory.getFirstButter();
        Action getSecondButterBuilt = trajectory.getSecondButter();
        Action getTransferFalseBuilt = trajectory.getTransfer(false);
        Action getTransferTrueBuilt = trajectory.getTransfer(true);
        Action getThirdButterBuilt = trajectory.getThirdButter();
        Action getHang2Built = trajectory.getHang();
        Action getHang3Built = trajectory.getHang();
        Action getHang4Built = trajectory.getHang();
        Action getHang5Built = trajectory.getHang();

        telemetry.clearAll();
        telemetry.addLine("ready :)");
        telemetry.update();
        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;
        if(side) {
            Actions.runBlocking(new SequentialAction(
//                    trajectoryLeft.getAllTrajectory()

                    trajectoryLeft.getInitialBasket(),
                    trajectoryLeft.getFirstButter(),
                    new ParallelAction(
                            trajectoryLeft.getButterAttachment(false, true),
                            trajectoryLeft.getBasket(false, true)
                    ),
                    trajectoryLeft.getSecondButter(),
                    new ParallelAction(
                            trajectoryLeft.getButterAttachment(false, true),
                            trajectoryLeft.getBasket(false, true)
                    ),
                    trajectoryLeft.getThirdButter(),
                    new ParallelAction(
                            trajectoryLeft.getButterAttachment(false, true),
                            trajectoryLeft.getBasket(false, true)
                    ),
                    trajectoryLeft.getSubmersible(sideway, attempt),
                    new ParallelAction(
                            trajectoryLeft.getButterAttachment(true, attempt),
                            trajectoryLeft.getBasket(true, attempt)
                    ),
                    trajectoryLeft.getPark()
            ));
        }  if(!side) {
            Actions.runBlocking(new SequentialAction(
//                    trajectory.getAlltrajectory()
//                    trajectory.testTransfer(),
                    getHang1Built,
                    getFirstButterBuilt,
                    new ParallelAction(
                            getSecondButterBuilt,
                            getTransferTrueBuilt
                    ),
                    getTransferFalseBuilt,
                    getThirdButterBuilt,
                    getHang2Built,
                    getHang3Built,
                    getHang4Built,
                    getHang5Built
            ));
        }
    }
}
