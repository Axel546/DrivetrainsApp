package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="AutonomousMecanumDrive", group="Linear Opmode")
// @Disabled
public class AutonomousMecanumDrive extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = 1, correction;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.dcMotor.get("front_left_motor");
        rightFrontMotor = hardwareMap.dcMotor.get(" front_right_motor");
        leftBackMotor = hardwareMap.dcMotor.get("back_left_motor");
        rightBackMotor = hardwareMap.dcMotor.get("back_right_motor");

        touch = hardwareMap.touchSensor.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Initializare IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        waitForStart();

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

        //facem o rotatie de 15 grade
        rotate(15,power);
        correction = checkDirection();

        //strafe stanga 20 cm
        leftFrontMotor.setTargetPosition(encoderDrivingTarget);
        rightFrontMotor.setTargetPosition(encoderDrivingTarget);
        leftBackMotor.setTargetPosition(encoderDrivingTarget);
        rightBackMotor.setTargetPosition(encoderDrivingTarget);

        leftFrontMotor.setPower(-1+correction);
        rightFrontMotor.setPower(1-correction);
        leftBackMotor.setPower(1+correction);
        rightBackMotor.setPower(-1-correction);

        //setam motoarele sa mearga pana la o anumita pozitie
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * unghiul curent.
     * returnam unghiul in grade. + = stanga, - = dreapta.
     */
    private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Vedem daca ne miscam drept, daca nu corectam pe cat posibil valoarea
     */
    private double checkDirection()
    {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotire stanga dreapta un anumit unghi. Nu se poate mai mult de 180 de grade. Totusi cred ca pentru o implementare, pentru mai mult de 180 se poate face de 2 ori rotatie
     */
    private void rotate(int degrees, double power)
    {
        double  leftfrontPower, rightfrontPower, leftbackPower, rightBackPower;

        resetAngle();

        // getAngle() returneaza + cand se roteste sens inver acelor de ceasornic (stanga) si + cand se roteste in sensul acelor (dreapta)

        if (degrees < 0)
        {   // rotire dreapta
            leftfrontPower = power;
            rightfrontPower = -power;
            leftbackPower = power;
            rightBackPower = -power;
        }
        else if (degrees > 0)
        {   // rotire stanga
            leftfrontPower = -power;
            rightfrontPower = +power;
            leftbackPower = -power;
            rightBackPower = +power;
        }
        else return;

        leftFrontMotor.setPower(leftfrontPower);
        rightFrontMotor.setPower(rightfrontPower);
        leftBackMotor.setPower(leftbackPower);
        rightBackMotor.setPower(rightBackPower);

        // roteste pana se termina
        if (degrees < 0)
        {//right turn
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        resetAngle();
    }
}