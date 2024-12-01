package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotverticalslides.DriveActions;
import org.firstinspires.ftc.teamcode.secondrobot.ArmActions;
@TeleOp(name = "Tele Op second robot", group = "Linear Opmode")
public class SecondaryTeleOp extends LinearOpMode {
    ArmActions armActions = null;

    double liftSpdMult = 0.8 ;

    @Override
    public void runOpMode() {
        armActions = new ArmActions(hardwareMap,telemetry);
        DcMotorEx armMotor = null;
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
         armActions.teleopArmmotor(gamepad2.b,gamepad2.a);

            telemetry.update();

        }
    }
}
