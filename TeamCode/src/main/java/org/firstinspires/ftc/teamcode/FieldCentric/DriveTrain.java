package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalVariables;

@Config
public class DriveTrain {
    DcMotorEx frontLeft; DcMotorEx rearLeft; DcMotorEx frontRight; DcMotorEx rearRight; IMU imu;
    double flActive = 1; double rlActive = 1; double frActive = 1; double rrActive = 1; double botHeading;

    public DriveTrain(DcMotorEx frontLeft, DcMotorEx rearLeft, DcMotorEx frontRight, DcMotorEx rearRight, IMU imu){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearRight = rearRight;
        this.rearLeft = rearLeft;
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.imu = imu;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        this.imu.initialize(parameters);
    }
    public void drive(double leftStickY, double leftStickX, double rightStickX, boolean reset, boolean percise, Pose2d curentPose){
        //take input from gamepad
        double y = -leftStickY;
        double x = leftStickX;
        double rx = rightStickX;

        if(percise){
            flActive = .5;
            rlActive = .5;
            frActive = .5;
            rrActive = .5;
        } else {
            flActive = 1;
            rlActive = 1;
            frActive = 1;
            rrActive = 1;
        }

//        if(reset){
//            imu.resetYaw();
//        }
        //take input from heading
//        if(GlobalVariables.autoStarted) {
//            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + curentPose.heading.toDouble();
//        }else {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        }
        //Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        //counter robot imperfect strafing
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double rearLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double rearRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower((frontLeftPower * flActive));
        frontRight.setPower((frontRightPower * frActive));
        rearRight.setPower((rearRightPower * rrActive));
        rearLeft.setPower((rearLeftPower * rlActive));
    }
}
