package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Championship.StrafeAction;
import org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Championship.TrajectoryLeftChampionship;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.*;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.LimeSweet;

import java.lang.Math;

@Autonomous(name = "ColorCalibration", group = "test")
public class ColorCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose = new Pose2d(0, 0, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose);
        LimeSweet lime = new LimeSweet(hardwareMap, telemetry, 0);
        lime.setInputs(new double[]{1.0, 2.0, 0.0, 4.0, 5.0, 6.0, 7.0, 8.0});

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

        StrafeAction strafeAction = new StrafeAction(drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront, lime, telemetry, horizontalGrabber, horizontalRoll,
                horizontalSlide, horizontalWrist, verticalGrabber, verticalHanger, verticalSlide, verticalWrist);

        TrajectoryLeftChampionship trajectory = new TrajectoryLeftChampionship(drive, pose, lime, strafeAction, telemetry, hardwareMap, horizontalGrabber, horizontalRoll,
                horizontalSlide, horizontalWrist, verticalGrabber, verticalHanger, verticalSlide, verticalWrist);

        lime.scanButter();

        telemetry.addLine("Ready");
        telemetry.update();

        while(opModeInInit()){
            lime.spicy();
        }

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        trajectory.getButterPose();
        Action getSubButter = trajectory.getSubButter().build();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        getSubButter,
//                        new SequentialAction(
////                                new SleepAction(.85), /*how long before triggers the close grabber*/
////                                trajectory.getTransfer()
//                        )),
                new SleepAction(20000)
        )));

    }
}
