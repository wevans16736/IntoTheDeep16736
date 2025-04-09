package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@Disabled
@TeleOp(name="1.ServoTest", group = "SecondRobot")
public class ServoTest extends LinearOpMode {

//    public static double VerticalGrabber = ConfigurationSecondRobot.verticalClose;
//    public static double VerticalHanger = ConfigurationSecondRobot.verticalHangIn;
//    public static double HorizontalGrabber = ConfigurationSecondRobot.horizontalGrabberOpen;
//    public static double HorizontalWrist = ConfigurationSecondRobot.horizontalWristIntake;
//    public static double HorizontalRoll = ConfigurationSecondRobot.flat;

//    public static double VerticalWrist2 = ConfigurationSecondRobot.verticalWristIntake;
//    public static double VerticalGrabber2 = ConfigurationSecondRobot.verticalClose;
//    public static double VerticalHanger2 = ConfigurationSecondRobot.verticalHangOut;
//    public static double HorizontalGrabber2 = ConfigurationSecondRobot.horizontalGrabberOpen;
//    public static double HorizontalWrist2 = ConfigurationSecondRobot.horizontalWristIntake;
//    public static double HorizontalRoll2 = ConfigurationSecondRobot.flat;



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, "test");
//        Servo VerticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
//        Servo VerticalHangServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_HANGER);
//
//        Servo HorizontalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
//        Servo HorizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
//        Servo HorizontalRollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);

        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        VerticalGrabberServo.setPosition(VerticalGrabber);
//        VerticalHangServo.setPosition(VerticalHanger);
//
//        HorizontalGrabberServo.setPosition(HorizontalGrabber);
//        HorizontalWristServo.setPosition(HorizontalWrist);
//        HorizontalRollServo.setPosition(HorizontalRoll);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("test", test.getCurrentPosition());
            telemetry.update();
//            VerticalWristServo.setPosition(VerticalWrist2);
//            VerticalGrabberServo.setPosition(VerticalGrabber2);
//            VerticalHangServo.setPosition(VerticalHanger2);
//
//            HorizontalGrabberServo.setPosition(HorizontalGrabber2);
//            HorizontalWristServo.setPosition(HorizontalWrist2);
//            HorizontalRollServo.setPosition(HorizontalRoll2);
        }
    }
}
