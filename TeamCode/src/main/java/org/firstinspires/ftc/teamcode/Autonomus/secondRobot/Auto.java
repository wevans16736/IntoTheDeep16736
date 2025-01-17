package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

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
        boolean side = false; //determine which side

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

        if(!side){
            pose = new Pose2d(9,-64,Math.toRadians(180));
            drive = new PinpointDrive(hardwareMap, pose);
        }else{
            pose = new Pose2d(-15,-64,Math.toRadians(180));
            drive = new PinpointDrive(hardwareMap, pose);
        }

        Trajectory trajectory = new Trajectory(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);

        TrajectoryLeft trajectoryLeft = new TrajectoryLeft(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist);

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;
        if(!side) {
            Actions.runBlocking(new SequentialAction(
                    trajectory.getAlltrajectory()

//                trajectory.getHang(),
//                //get all the butter
//                trajectory.getFirstButter(),
//                new ParallelAction(
//                        trajectory.ButterTransferAttachment(),
//                        trajectory.getFirstButterDropOff()
//                ),
//                trajectory.getSecondButter(),
//                new ParallelAction(
//                        trajectory.ButterTransferAttachment(),
//                        trajectory.getSecondButterDropOff()
//                ),
//                trajectory.getThirdButter(),
//                trajectory.getThirdButterDropOff(),
//                //postHang
//                trajectory.getPostHang(),
//                new SleepAction(2)
            ));
        } if(side) {
            Actions.runBlocking(new SequentialAction(
                    trajectoryLeft.getAllTrajectory()

//                    trajectoryLeft.getInitialBasket(),
//                    trajectoryLeft.getFirstButter(),
//                    new ParallelAction(
//                            trajectoryLeft.getButterAttachment(),
//                            trajectoryLeft.getBasket()
//                    ),
//                    trajectoryLeft.getSecondButter(),
//                    new ParallelAction(
//                            trajectoryLeft.getButterAttachment(),
//                            trajectoryLeft.getBasket()
//                    ),
//                    trajectoryLeft.getThirdButter(),
//                    new ParallelAction(
//                            trajectoryLeft.getButterAttachment(),
//                            trajectoryLeft.getBasket()
//                    ),
//                    trajectoryLeft.getPark()
            ));
        }
    }
}
