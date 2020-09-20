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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutonomousMecanumDrive", group="Linear Opmode")
// @Disabled
public class AutonomousMecanumDriveComplex extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = 1, correction;
    private DistanceSensor sensorRange;
    double cmtowall = 10;

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

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        waitForStart();

        //de cate ori trebuie sa se roteasca rotile pentru a parcurge 20 cm
        //distanta pe care o parcurgi cu o roata este circumferinta ei
        double distance = 20;
        double circumference = 3.14 * 10; //pi*diametru
        double rotationsNeeded = distance/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded * 383.6); //rotatii * tick_counts

        runWithEncoders(encoderDrivingTarget,1,1);

        full(1,1);
        if(sensorRange.getDistance(DistanceUnit.CM) < cmtowall)
        {
            full(0,0);
        }


        //facem o rotatie la 15 grade
        rotateTo(15);
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

    public static double cmp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void back (double left, double right){
        left = cmp(left, -1, 1);
        right = cmp(right, -1, 1);

        leftBackMotor.setPower(left);
        rightBackMotor.setPower(right);
    }

    public void front (double left, double right){
        left = cmp(left, -1, 1);
        right = cmp(right, -1, 1);

        leftFrontMotor.setPower(left);
        rightFrontMotor.setPower(right);
    }

    public void full (double i, double j){
        back(i, j);
        front(i, j);
    }

    private final double P = 35,
            I = 5,
            D = 70;

    private final double MAX_ERROR = 5;

    private void rotateTo(double direction) {
        double angle = getAngle();
        double error  = direction - angle;
        double lastError = error;
        boolean arrived = false;
        while (opModeIsActive() && arrived == false){
            //PID
            double  motorCorrection = (((P * error) + (I * (error + lastError)) + D * (error - lastError))) / 100;

            full(- motorCorrection, motorCorrection);

            lastError = error;
            angle = getAngle();
            error = direction - angle;

            telemetry.addData("Angle", "%f", angle);
            telemetry.addData("Error", "%f", error);
            telemetry.update();

            if(Math.abs(error) <= MAX_ERROR && arrived == false){
                arrived = true;
            }
        }

        telemetry.update();
    }

    private void stopResetEncoders()
    {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTarget(int target)
    {
        leftFrontMotor.setTargetPosition(target);
        rightFrontMotor.setTargetPosition(target);
        leftBackMotor.setTargetPosition(target);
        rightBackMotor.setTargetPosition(target);
    }

    private void runToPos()
    {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void checkIsBusy()
    {
        while(leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy())
        {
        }
    }

    private void runWithEncoders(int target,double i, double j)
    {
        stopResetEncoders();
        setTarget(target);
        full(i,j);
        runToPos();
        checkIsBusy();
    }

}