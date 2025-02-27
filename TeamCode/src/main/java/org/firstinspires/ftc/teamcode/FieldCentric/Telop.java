package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FieldCentric")
public class Telop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    VerticalSlideRR verticalSlide; VerticalWristRR verticalWrist; VerticalGrabberRR verticalGrabber; VerticalHangerRR verticalHanger;
    HorizontalSlideRR horizontalSlide; HorizontalRollRR horizontalRoll; HorizontalGrabberRR horizontalGrabber; HorizontalWristRR horizontalWrist;
    DriveTrain driveTrain; Pose2d currentPose; Attachment attachment;
    DcMotorEx leftFront; DcMotorEx leftBack; DcMotorEx rightBack; DcMotorEx rightFront; boolean manual = true;

//    DriveActions driveActions;
    PinpointDrive drive;
    @Override
    public void init() {
        //all of these class is under Configuration.secondRobot
        verticalSlide = new VerticalSlideRR(hardwareMap);
        verticalWrist = new VerticalWristRR(hardwareMap , true);
        verticalGrabber = new VerticalGrabberRR(hardwareMap);

        horizontalSlide = new HorizontalSlideRR(hardwareMap);
        horizontalRoll = new HorizontalRollRR(hardwareMap);
        horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        horizontalWrist = new HorizontalWristRR(hardwareMap);
        verticalHanger = new VerticalHangerRR(hardwareMap);

        if(GlobalVariables.autoStarted){
            this.currentPose = GlobalVariables.currentPose;
            this.drive = new PinpointDrive(hardwareMap,currentPose);
            GlobalVariables.autoStarted = false;
        } else {
            this.currentPose = new Pose2d(-15, -64, Math.toRadians(180));
            this.drive = new PinpointDrive(hardwareMap, currentPose);
        }
        IMU imu = hardwareMap.get(IMU.class, ConfigConstants.IMU);

        //setup the drive train
        leftFront = this.drive.leftFront;
        leftBack = this.drive.leftBack;
        rightBack = this.drive.rightBack;
        rightFront = this.drive.rightFront;

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack, imu, drive);

//        driveActions = new DriveActions(telemetry, hardwareMap);

        attachment = new Attachment(verticalSlide, verticalWrist, verticalGrabber,
                horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist, verticalHanger,
                runningActions, dash, drive);
    }
    boolean percise = false;
    @Override
    public void loop() {
        //update time
        attachment.updateTime(System.currentTimeMillis());
        //verticalGrabber
        attachment.verticalGrabber(gamepad1.right_bumper);
        //horizontal slide
        attachment.horizontalSlide(gamepad1.triangle);
        //horizontal grabber
        attachment.horizontalGrabber(gamepad1.right_trigger);
        //vertical slide
        attachment.verticalSlide(gamepad1.left_trigger, gamepad1.left_bumper);
        //transfer wrist
        attachment.transferWrist(gamepad1.circle);
        //hook
        attachment.hook(gamepad1.dpad_up);
        //roll
        attachment.roll(gamepad1.dpad_left);
        //percise mode
        percise = attachment.percise(gamepad1.square);
        //manual slide override
//        attachment.verticalOverride(gamepad1.cross, gamepad1.dpad_up, gamepad1.dpad_down);
        //basket
        if(!manual) {
            drive.updatePoseEstimate();
            attachment.driveBasket(gamepad1.cross);
            telemetry.addData("x", drive.getLastPinpointPose().position.x);
            telemetry.addData("y", drive.getLastPinpointPose().position.y);
            telemetry.addData("heading",  Math.toDegrees(drive.getLastPinpointPose().heading.toDouble()));
            telemetry.update();
        }


//        driveActions.drive(
//                //joystick controlling strafe
//                (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)),
//                //joystick controlling forward/backward
//                (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)),
//                //joystick controlling rotation
//                -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

//        field centric robot control
        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_bumper){
            drive.updatePoseEstimate();
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.options, percise,drive.getLastPinpointPose().heading.toDouble());
            manual = true;
        } else{
            driveTrain.drive(0,0,0, false, percise, currentPose.heading.toDouble());
            manual = false;
        }

        //update runing actions
        attachment.updateAction();
//        RobotLog.d("robot orientation: " + (driveTrain.botHeading));
    }
}