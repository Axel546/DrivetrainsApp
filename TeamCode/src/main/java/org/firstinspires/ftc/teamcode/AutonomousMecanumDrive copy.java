package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousMecanumDrive", group="Linear Opmode")
// @Disabled
public class AutonomousMecanumDrive extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.dcMotor.get("front_left_motor");
        rightFrontMotor = hardwareMap.dcMotor.get(" front_right_motor");
        leftBackMotor = hardwareMap.dcMotor.get("back_left_motor");
        rightBackMotor = hardwareMap.dcMotor.get("back_right_motor");

        //merge inainte 20cm
        //resetam encoderele
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //de cate ori trebuie sa se roteasca rotile pentru a parcurge 20 cm
        //distantape care o parcurgi cu o roata este circumferinta ei
        double circumference = 3.14 * 10; //pi*diametru
        double rotationsNeeded = 20/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded * 383.6); //rotatii * tick_counts

        leftFrontMotor.setTargetPosition(encoderDrivingTarget);
        rightFrontMotor.setTargetPosition(encoderDrivingTarget);
        leftBackMotor.setTargetPosition(encoderDrivingTarget);
        rightBackMotor.setTargetPosition(encoderDrivingTarget);

        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(1);
        leftBackMotor.setPower(1);
        rightBackMotor.setPower(1);

        //setam motoarele sa mearga pana la o anumita pozitie
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy())
        {
            //in principiu nu facem nimic dar printez pe ecran ce se intampla pentru un eventual debug
            telemetry.addData("Path", "Driving 20 cm");
            telemetry.update();
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}