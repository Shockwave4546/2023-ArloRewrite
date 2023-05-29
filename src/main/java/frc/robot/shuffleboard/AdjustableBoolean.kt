package frc.robot.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets

class AdjustableBoolean(name: String, def: Boolean, pos: Pos2D = Pos2D.NO_POSITION) : AdjustableValue<Boolean>(name, def, {
  it.withWidget(BuiltInWidgets.kToggleButton)
  if (pos != Pos2D.NO_POSITION) it.withPosition(pos.x, pos.y)
  it
}) {
  override fun set(value: Boolean) {
    entry.setBoolean(value)
  }

  override fun get() = entry.getBoolean(def)
}