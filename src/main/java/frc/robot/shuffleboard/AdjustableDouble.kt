package frc.robot.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import kotlin.math.max
import kotlin.math.min

class AdjustableDouble(name: String, def: Double, pos: Pos2D = Pos2D.NO_POSITION, private val minValue: Double=Double.MIN_VALUE, private val maxValue: Double=Double.MAX_VALUE) : AdjustableValue<Double>(name, def, {
  it.withWidget(BuiltInWidgets.kTextView)
  if (pos != Pos2D.NO_POSITION) it.withPosition(pos.x, pos.y)
  it
}) {
  override fun set(value: Double) {
    entry.setDouble(value)
  }

  override fun get() = max(minValue, min(entry.getDouble(def), maxValue))
}