package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import java.util.Queue;
import java.util.LinkedList;
import java.lang.Object;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

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
    double eps = 1.e-14;

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

        runUntilWallLessThan(cmtowall);


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

    private final double MAX_ERROR = 5;

    private void rotateTo(double direction)
    {
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();
        Queue<Pair<Double, Double>> q = new LinkedList<>();
        double firstgood, lastgood;
        double wait = 10; //milliseconds you want to stay there
        firstgood = 0;
        lastgood = 0;
        double angle = getAngle();
        double error = direction - angle;
        double lastError = 0, tlastError = 0;
        double motorpower;
        double sum = 0;
        double P = 35, I = 5, D = 70;
        while (firstgood - lastgood < wait)
        {
            error = direction - getAngle();
            double sec = mRuntime.milliseconds();
            if (abs(error) < eps)
                if (firstgood == 0)
                    firstgood = sec;
                else
                    lastgood = sec;
            else {
                firstgood = 0;
                lastgood = 0;
            }

            Pair<Double, Double> x = new Pair<>(error, sec);
            q.add(x);
            sum += x.first;
            while (sec - q.peek().second > wait) {
                sum -= q.peek().first;
                q.remove();
            }
            motorpower = P * error + I * sum + D * (error - lastError) * (sec - tlastError);
            full(-motorpower, motorpower);
            lastError = error;
            tlastError = sec;
        }
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

    private void runUntilWallLessThan(double cmtowall)
    {
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();
        Queue<Pair<Double,Double>> q = new LinkedList<>();
        double firstgood,lastgood;
        double wait = 10; //milliseconds you want to stay there
        firstgood = 0;
        lastgood = 0;
        double sum = 0;
        double error = sensorRange.getDistance(DistanceUnit.CM) - cmtowall;
        double lastError = 0,tlastError = 0;
        double motorpower;
        double P = 35, I = 5, D = 70;
        while(firstgood-lastgood < wait)
        {
            error = sensorRange.getDistance(DistanceUnit.CM) - cmtowall;
            double sec = mRuntime.milliseconds();
            if(abs(error) < eps)
                if(firstgood == 0)
                    firstgood = sec;
                else
                    lastgood = sec;
            else
            {
                firstgood = 0;
                lastgood = 0;
            }

            Pair<Double, Double> x = new Pair<>(error, sec);
            q.add(x);
            sum += x.first;
            while(sec - q.peek().second > wait)
            {
                sum -= q.peek().first;
                q.remove();
            }
            motorpower = P * error + I * sum + D * (error-lastError)*(sec - tlastError);
            full(motorpower,motorpower);
            lastError = error;
            tlastError = sec;
        }
    }

    private void runRotateUntilLessThan(double cmtowall, double direction)
    {
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();
        Queue<Pair<Double,Double>> q = new LinkedList<>();
        double firstgood,lastgood;
        double wait = 10; //milliseconds you want to stay there
        firstgood = 0;
        lastgood = 0;
        double sum = 0;
        double error = sensorRange.getDistance(DistanceUnit.CM) - cmtowall;
        double lastError = 0,tlastError = 0;
        double motorpower;
        double P = 35, I = 5, D = 70;
        ElapsedTime mRuntime1 = new ElapsedTime();
        mRuntime.reset();
        Queue<Pair<Double, Double>> q1 = new LinkedList<>();
        double firstgood1, lastgood1;
        double wait1 = 10; //milliseconds you want to stay there
        firstgood1 = 0;
        lastgood1 = 0;
        double angle = getAngle();
        double error1 = direction - angle;
        double lastError1 = 0, tlastError1 = 0;
        double motorpower1;
        double sum1 = 0;
        while(firstgood-lastgood < wait && firstgood1 - lastgood1 < wait1)
        {
            error = sensorRange.getDistance(DistanceUnit.CM) - cmtowall;
            double sec = mRuntime.milliseconds();
            if(abs(error) < eps)
                if(firstgood == 0)
                    firstgood = sec;
                else
                    lastgood = sec;
            else
            {
                firstgood = 0;
                lastgood = 0;
            }
            error1 = direction - getAngle();
            double sec1 = mRuntime1.milliseconds();
            if (abs(error1) < eps)
                if (firstgood1 == 0)
                    firstgood1 = sec1;
                else
                    lastgood1 = sec1;
            else {
                firstgood1 = 0;
                lastgood1 = 0;
            }

            Pair<Double, Double> x = new Pair<>(error, sec);
            q.add(x);
            sum += x.first;
            while(sec - q.peek().second > wait)
            {
                sum -= q.peek().first;
                q.remove();
            }
            Pair<Double, Double> x1 = new Pair<>(error1, sec1);
            q1.add(x1);
            sum1 += x1.first;
            while (sec1 - q1.peek().second > wait1) {
                sum1 -= q1.peek().first;
                q1.remove();
            }

            motorpower = P * error + I * sum + D * (error-lastError)*(sec - tlastError);
            motorpower1 = P * error1 + I * sum1 + D * (error1 - lastError1) * (sec1 - tlastError1);
            full(motorpower - motorpower1, motorpower + motorpower1);
            lastError = error;
            tlastError = sec;
            lastError1 = error1;
            tlastError1 = sec1;
        }
    }
}