package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.Gear;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Turret is the super-subsystem for both the Gear and Launcher subsystems
 * Combines both subsystems for use in teleOps and autoOps
 * Should streamline management of the turret.
 */
public class Turret {

    Gear gear;
    Launcher launcher;

    public Turret()
    {
        //Should implement both a gear and launcher.
    }
}
