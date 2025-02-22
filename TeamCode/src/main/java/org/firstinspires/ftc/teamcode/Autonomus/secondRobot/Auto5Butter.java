package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Trajectory;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.DetectBlockActions;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

@Autonomous(name = "rotation")
public class Auto5Butter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;
//        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, ConfigConstants.COLOR_SENSOR);
        boolean side = false;

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        if(!side){
            pose = new Pose2d(9,-64,Math.toRadians(90));
            drive = new PinpointDrive(hardwareMap, pose);
        }else{
            pose = new Pose2d(-15,-64,Math.toRadians(180));
            drive = new PinpointDrive(hardwareMap, pose);
        }

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

        DetectBlockActions vision = new DetectBlockActions(hardwareMap);

        RobotSensor robotSensor = new RobotSensor(telemetry, drive, vision);

        Trajectory trajectory = new Trajectory(drive, pose, verticalSlide, verticalWrist, verticalGrabber,
                verticalHanger, horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist, robotSensor);

        //build everything
        Action hang1 = trajectory.getHang();
        Action butter1 = trajectory.getButter();
        Action attachment1 = trajectory.getAttachment();
        Action butter2 = trajectory.getButter();
        Action attachment2 = trajectory.getAttachment();
        Action butter3 = trajectory.getButter();
        Action hang2 = trajectory.getHang();
        Action Human1 = trajectory.getHuman();
        Action hang3 = trajectory.getHang();
        Action Human2 = trajectory.getHuman();
        Action  hang4 = trajectory.getHang();
        Action  Human3 = trajectory.getHuman();
        Action hang5 = trajectory.getHang();
        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        GlobalVariables.autoStarted = true;

        Actions.runBlocking(new SequentialAction(
                hang1,
                butter1,
                new ParallelAction(
                        attachment1,
                        butter2
                ),
                attachment2,
                butter3,
                hang2,
                Human1,
                hang3,
                Human2,
                hang4,
                Human3,
                hang5
        ));
        GlobalVariables.currentPose = drive.getLastPinpointPose();
    }
}