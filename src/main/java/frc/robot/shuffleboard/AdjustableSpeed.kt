package frc.robot.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets

class AdjustableSpeed(name: String, def: Double, pos: Pos2D = Pos2D.NO_POSITION) : AdjustableValue<Double>(name, def, {
  it.withWidget(BuiltInWidgets.kNumberSlider).withProperties(mapOf("min" to "-1", "max" to "1"))
  if (pos != Pos2D.NO_POSITION) it.withPosition(pos.x, pos.y)
  it
}) {
  override fun set(value: Double) {
    entry.setDouble(value)
  }

  override fun get() = entry.getDouble(def)
}