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
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Beans!", group="Linear Opmode")

public class Beans extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor arm = null;
    private DcMotor modelX = null;
    private DcMotor hand1 = null;
    private DcMotor hand2 = null;
    private Servo claw = null;
    private Servo finger1 = null;
    private Servo finger2 = null;
        
    //Gyro Testing Stuff
    protected BNO055IMU imu = null;
    private double zero_heading = 0.0;
    private double zero_y = 0.0;
    private Orientation angles = null;

    @Override
    public void runOpMode() {
        double mod = 1.0;
        telemetry.addData("Status", "Beans has been initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        modelX = hardwareMap.get(DcMotor.class, "ModelX");
        finger1 = hardwareMap.get(Servo.class, "finger1");
        finger2 = hardwareMap.get(Servo.class, "finger2");
        hand1 = hardwareMap.get(DcMotor.class, "hand1");
        hand2 = hardwareMap.get(DcMotor.class, "hand2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
            
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive_0.setDirection(DcMotor.Direction.REVERSE);
        rightDrive_0.setDirection(DcMotor.Direction.FORWARD);
        leftDrive_1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive_1.setDirection(DcMotor.Direction.FORWARD);
        modelX.setDirection(DcMotor.Direction.FORWARD);
        hand1.setDirection(DcMotor.Direction.FORWARD);
        hand2.setDirection(DcMotor.Direction.REVERSE);
        
        
        leftDrive_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        arm.setDirection(DcMotor.Direction.FORWARD);
        claw.setPosition(1.0);
        double servoPosition = 0.0;
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        double strafe = 0.0;
        double vbrake;
        double actLF = 0.0;
        double actLB = 0.0;
        double actRF = 0.0;
        double actRB = 0.0;
        double delta = 0.3;
        double targetLF, targetLB, targetRF, targetRB;
        double finger_rear = 0.0;
        double finger_rear_static = 0.0;
        double claw_pos = 1.0;
        boolean claw_pressing = false;
        
        //Gyro Config
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        //params.accelerationIntegerationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
        zeroHeading();
        
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
                strafe = -gamepad1.left_stick_x - gamepad1.right_stick_x;
            }
            // Conditional statement to allow for more percise and slow movement.
            if(gamepad1.right_bumper) 
            {
                leftPower = leftPower*0.5;
                rightPower = rightPower*0.5;
                strafe = strafe*0.5;
            }
            else{
                leftPower *= 1.0;
                rightPower *= 1.0;
                strafe *= 1.0;
            }
            
            vbrake = 1.0 - gamepad1.right_trigger;
            leftPower *= vbrake;
            rightPower *= vbrake;
            
            telemetry.addData("Braking", "Braking at: (%2.0f) %%", (vbrake*100));
            
            targetLF = leftPower - strafe;
            if(actLF < targetLF - delta){
                actLF += delta;
            }
            else if(actLF > targetLF + delta){
                actLF -= delta;
            }
            else{
                actLF = targetLF;
            }
            targetLB = leftPower + strafe;
            if(actLB < targetLB - delta){
                actLB += delta;
            }
            else if(actLB > targetLB + delta){
                actLB -= delta;
            }
            else{
                actLB = targetLB;
            }
            targetRF = rightPower + strafe;
            if(actRF < targetRF - delta){
                actRF += delta;
            }
            else if(actRF > targetRF + delta){
                actRF -= delta;
            }
            else{
                actRF = targetRF;
            }
            targetRB = (rightPower - strafe)*0.7;
            if(actRB < targetRB - delta){
                actRB += delta;
            }
            else if(actRB > targetRB + delta){
                actRB -= delta;
            }
            else{
                actRB = targetRB;
            }
            
            // Send calculated power to wheels
            leftDrive_0.setPower(actLF);
            rightDrive_0.setPower(actRF);
            leftDrive_1.setPower(actLB);
            rightDrive_1.setPower(actRB);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Heading", "Z %2.3f, Y %2.3f", getAngle(), (angles.secondAngle-zero_y+360)%360.0);
            telemetry.update();
            
            if(gamepad1.left_bumper){
                finger1.setPosition(0.0);
                finger2.setPosition(1.0);
            }
            else if(gamepad1.left_trigger > 0.1){
                finger1.setPosition(0.9);
                finger2.setPosition(0.1);
            }
            telemetry.addData("Fingers (Front)", "finger1 (%.2f), finger2 (%.2f)", finger1.getPosition(), finger2.getPosition());
            
            finger_rear = 0.2*gamepad2.left_trigger * (gamepad2.left_bumper?-1.0:1.0) + finger_rear_static;
            if(gamepad2.left_bumper && gamepad2.left_trigger > 0.0){
                finger_rear_static = -0.1;
            }
            else if(gamepad2.left_trigger > 0.0){
                finger_rear_static = 0.0;
            }
            hand1.setPower(finger_rear);
            hand2.setPower(finger_rear);
            telemetry.addData("Fingers (Rear)", "power: %.2f, static: %.2f", finger_rear, finger_rear_static);
            
            arm.setPower(gamepad2.right_stick_y*0.4-0.12);
            telemetry.addData("Arm", "Arm %.2f", -gamepad2.right_stick_y);
            
            if(gamepad2.y){
                modelX.setPower(0.2);
            }
            else if(gamepad2.a){
                modelX.setPower(-0.2);
            }
            else{
                modelX.setPower(0.0);
            }
            
            if(gamepad2.right_bumper && !claw_pressing){
                claw_pos = claw_pos>0.5 ? 0.3 : 0.9;
                claw_pressing = true;
            } else if(!gamepad2.right_bumper && claw_pressing) {
                claw_pressing = false;
            }
            claw.setPosition(claw_pos);
            telemetry.addData("LMotors0", leftDrive_0.getCurrentPosition());
            telemetry.addData("Servos", "claw (%.2f)", claw.getPosition());
            
        }
    }
    
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading+360) % 360.0;
    }
    
    public void zeroHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading  = angles.firstAngle;
        zero_y = angles.secondAngle;
    }
}
