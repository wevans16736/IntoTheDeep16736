package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalSlideActions;

@TeleOp(name="SlideTest", group = "SecondRobot")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx horizontalSlide = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_ARM);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx verticalSlide = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("vertical Slide Position", verticalSlide.getCurrentPosition());
            telemetry.addData("Horizontal Slide Position", horizontalSlide.getCurrentPosition());

            telemetry.update();
        }

    }
 }