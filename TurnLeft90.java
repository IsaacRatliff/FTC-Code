package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Turn Left 90 degrees")
public class TurnLeft90 extends AutoOp {

    @Override
    public void runOpMode(){
        initialize(hardwareMap, telemetry);
        waitForStart();
        pause(0.5);
        left_gyro(1);
        armUp();
        right_gyro(1);
        armDown();
        pause(3);
    }
}
