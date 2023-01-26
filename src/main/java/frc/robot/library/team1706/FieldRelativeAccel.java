package frc.robot.library.team1706;
/**
 * this class is copied from team1796's code of the 2022 season.
 * FieldRelativeAccel描述了车子以场地为参考系的加速度，包含以场地为坐标系的前后左右的加速度，以及车子底盘的旋转加速度。
 */
public class FieldRelativeAccel {
    public double m_ax;
    public double m_ay;
    public double m_alpha;

    public FieldRelativeAccel(double _ax, double _ay, double _Alpha) {
        this.m_ax = _ax;
        this.m_ay = _ay;
        this.m_alpha = _Alpha;
    }

    public FieldRelativeAccel(FieldRelativeSpeed _NewSpeed, FieldRelativeSpeed _OldSpeed, double _Time) {
        this.m_ax = (_NewSpeed.vx - _OldSpeed.vx) / _Time;
        this.m_ay = (_NewSpeed.vy - _OldSpeed.vy) / _Time;
        this.m_alpha = (_NewSpeed.omega - _OldSpeed.omega) / _Time;

        if (Math.abs(this.m_ax) > 6.0) {
            this.m_ax = 6.0 * Math.signum(this.m_ax);
        }
        if (Math.abs(this.m_ay) > 6.0) {
            this.m_ay = 6.0 * Math.signum(this.m_ay);
        }
        if (Math.abs(this.m_alpha) > 4 * Math.PI) {
            this.m_alpha = 4 * Math.PI * Math.signum(this.m_alpha);
        }
    }

    public FieldRelativeAccel() {
        this.m_ax = 0.0;
        this.m_ay = 0.0;
        this.m_alpha = 0.0;
    }

}
