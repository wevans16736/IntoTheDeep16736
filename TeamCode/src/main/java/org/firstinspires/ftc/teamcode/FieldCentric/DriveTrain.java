package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Config
public class DriveTrain {
    DcMotorEx frontLeft; DcMotorEx rearLeft; DcMotorEx frontRight; DcMotorEx rearRight; IMU imu; PinpointDrive drive;
    double active; double botHeading;

    public DriveTrain(DcMotorEx frontLeft, DcMotorEx rearLeft, DcMotorEx frontRight, DcMotorEx rearRight, IMU imu, PinpointDrive drive){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearRight = rearRight;
        this.rearLeft = rearLeft;
        this.imu = imu;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        this.imu.initialize(parameters);
        this.drive = drive;
    }
    public void drive(double leftStickY, double leftStickX, double rightStickX, boolean reset, boolean percise, double heading){
        //take input from gamepad
        double y = -leftStickY;
        double x = leftStickX;
        double rx = rightStickX;

        if(percise){
           active = .5;
        } else {
           active = 1;
        }

//        if(reset){
//            imu.resetYaw();
//        }
        //take input from heading
//        if(GlobalVariables.autoStarted) {
//            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + curentPose.heading.toDouble();
//        }else {
//            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        }
        botHeading = heading - Math.toRadians(90);
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

        frontLeft.setPower((frontLeftPower * active));
        frontRight.setPower((frontRightPower * active));
        rearRight.setPower((rearRightPower * active));
        rearLeft.setPower((rearLeftPower * active));
    }
}
