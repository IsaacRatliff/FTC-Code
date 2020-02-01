package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="15 Pt: Reposition/Park (Red, Near)", group="Linear Opmode")

public class ModelXAuton extends AutoOp {
    
    /**
    Starting Position:
        Just right of the right edge of the foundation.
    **/

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(2.0);
        modelXDown();
        backward(2.0);
        modelXUp();
        forward(1.0);
        left(1);
        forward(1.0);
        // end
    }
}
