package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Decapricated;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Disabled
@Autonomous(name = "AutoLeft")
public class AutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;
//        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, ConfigConstants.COLOR_SENSOR);

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
        boolean side = true; //determine which side
        boolean sideway = false; //determine roll if left chosen
        boolean attempt = true; //determine if we even want to try blind pick
//        telemetry.clearAll();
//        telemetry.addLine("Choose which side?");
//        telemetry.addLine("Left Side: dpad-left");
//        telemetry.addLine("Right Side: dpad-right");
//        telemetry.update();
//        boolean wasPress = false;
//        while(!(gamepad1.dpad_left || gamepad1.dpad_right)){
//            if(gamepad1.dpad_left){
//                side = true;
//                wasPress = true;
//            } else if (gamepad1.dpad_right) {
//                side = false;
//            }
//        }
//        sleep(2000);
//        if (side) {
//            while (!(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up)) {
//                telemetry.clearAll();
//                telemetry.addLine("left side chosen");
//                telemetry.addData("attempt? dpad-left", attempt);
//                telemetry.addData("sideway? dpad-right", sideway);
//                telemetry.addLine("Good? dpad-up");
//                telemetry.update();
//                if (gamepad1.dpad_left) {
//                    attempt = false;
//                } else if (gamepad1.dpad_right) {
//                    sideway = true;
//                }
//            }
//        }
        if (!side) {
            pose = new Pose2d(9, -64, Math.toRadians(90));
            drive = new PinpointDrive(hardwareMap, pose);
        } else {
            pose = new Pose2d(-15, -64, Math.toRadians(180));
            drive = new PinpointDrive(hardwareMap, pose);
        }

        org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Decapricated.Trajectory trajectory = new Trajectory(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
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
        telemetry.addData("side", side);
        telemetry.addData("attempt", attempt);
        telemetry.addData("sideway", sideway);
        telemetry.update();


        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;
        if (side) {
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
        }
        if (!side) {
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
