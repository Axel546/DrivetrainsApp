package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test", group = "Test")
//@TeleOp()
public class Test extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        ///Secventa de initializare
        DcMotor leftFrontMotor = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor rightFrontMotor = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor leftBackMotor = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor rightBackMotor = hardwareMap.dcMotor.get("back_right_motor");


        waitForStart();

        while(opModeIsActive())
        {
            double fata_spate = gamepad1.left_stick_y;
            double stanga_dreapta = gamepad1.left_stick_x;
            double rotatie = gamepad1.right_stick_x;

            leftFrontMotor.setPower(fata_spate - stanga_dreapta - rotatie);
            rightFrontMotor.setPower(fata_spate + stanga_dreapta - rotatie);
            leftBackMotor.setPower(fata_spate + stanga_dreapta + rotatie);
            rightBackMotor.setPower(fata_spate - stanga_dreapta + rotatie);

        }
    }
}
