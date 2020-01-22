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


import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Beans!w/motorsync", group="Linear Opmode")

public class BeansMotorAligned extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor arm = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    
    
    @Override
    public void runOpMode() {
        double mod = 1.0;
        telemetry.addData("Status:", "Beans has been initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        
        leftDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        //claw1.getController().pwmEnable();
        //claw2.getController().pwmEnable();
            

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        claw1.setPosition(0.0);
        claw2.setPosition(0.0);
        double servoPosition = 0.0;
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double strafe = 0.0;
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Tank Mode uses one stick to control each wheel.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            if(gamepad1.dpad_right)
            {
                strafe = -1.0;
            }
            else if(gamepad1.dpad_left)
            {
                strafe = 1.0;
            }
            else{
                strafe = 0.0;
            }
            // Conditional statement to allow for more percise and slow movement.
           /* if(gamepad1.right_bumper) 
            {
                leftPower = leftPower*1.0;
                rightPower = rightPower*1.0;
                strafe = strafe*1.0;
            }
            */
            // Send calculated power to wheels
            leftDrive_0.setPower((leftPower - strafe));
            rightDrive_0.setPower(rightPower + strafe);
            leftDrive_1.setPower(leftPower + strafe);
            rightDrive_1.setPower(rightPower - strafe);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            

            if (gamepad1.left_bumper)
            {
                arm.setPower(.5);
            }
            else{
                arm.setPower(gamepad1.left_trigger * 0.5);
            }
            if (gamepad1.right_bumper)
            {
                arm.setPower(-.5);
            }
            else{
                arm.setPower(gamepad1.right_trigger * -0.5);
            }
            if(gamepad1.x){
                claw1.setPosition(0.25);
                claw2.setPosition(0.75);
            }
            else if(gamepad1.y){
                claw1.setPosition(0.75);
                claw2.setPosition(0.25);
            }
            telemetry.addData("Servos", "claw1 (%.2f), claw2 (%.2f)", claw1.getPosition(), claw2.getPosition());
        }
    }
    
}
                                                                                                                                                                                                                                                                
