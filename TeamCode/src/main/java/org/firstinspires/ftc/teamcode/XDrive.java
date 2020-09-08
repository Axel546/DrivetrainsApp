/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math.*;


@TeleOp(name="XDrive", group="Linear Opmode")
//@Disabled
public class XDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor m1Front = null;
    DcMotor m2Right = null;
    DcMotor m3Back = null;
    DcMotor m4Left = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        m1Front  = hardwareMap.get(DcMotor.class, "m1");
        m2Right = hardwareMap.get(DcMotor.class, "m2");
        m3Back = hardwareMap.get(DcMotor.class, "m3");
        m4Left = hardwareMap.get(DcMotor.class, "m4");

        m1Front.setPower(0.0);
        m2Right.setPower(0.0);
        m3Back.setPower(0.0);
        m4Left.setPower(0.0);

        m1Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double frontBackPower;
            double leftRightPower;
            double spinPower = -gamepad1.right_stick_x;

            if(Math.abs(spinPower) > .1) //if the right joystick is pressed then we want the motors to spin
            {
                m1Front.setPower(spinPower); //m1 and m3 wheels drive us to left and right
                m2Right.setPower(spinPower); //m2 and m4 wheels drive us to front and back
                m3Back.setPower(spinPower);
                m4Left.setPower(spinPower);
            }
            else //if there is no power
            {//Run wheels in tank mode
                frontBackPower = -gamepad1.left_stick_y;
                leftRightPower = -gamepad1.left_stick_x;

                m1Front.setPower(leftRightPower); //m1 and m3 wheels drive us to left and right
                m2Right.setPower(frontBackPower); //m2 and m4 wheels drive us to front and back
                m3Back.setPower(-leftRightPower);
                m4Left.setPower(-frontBackPower);
            }

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
