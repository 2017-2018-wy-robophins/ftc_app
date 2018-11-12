import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import static org.hamcrest.CoreMatchers.is;

@RunWith(RobolectricTestRunner.class)
@Config(sdk = 19)
public class NavigationalStateTests {
    @Test
    public void navigationalState_CorrectStartAngle_RedLeft() {
        Assert.assertThat(new NavigationalState(FieldConstants.redLeftStartLocation).get_heading(), is(-45f));
    }

    @Test
    public void navigationalState_CorrectStartAngle_RedRight() {
        Assert.assertThat(new NavigationalState(FieldConstants.redRightStartLocation).get_heading(), is(45f));
    }

    @Test
    public void navigationalState_CorrectStartAngle_BlueLeft() {
        Assert.assertThat(new NavigationalState(FieldConstants.blueLeftStartLocation).get_heading(), is(135f));
    }

    @Test
    public void navigationalState_CorrectStartAngle_BlueRight() {
        Assert.assertThat(new NavigationalState(FieldConstants.blueRightStartLocation).get_heading(), is(-134.99998f));
    }
}
