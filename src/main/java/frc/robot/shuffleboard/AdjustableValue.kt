package frc.robot.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget
import frc.robot.Tabs
import java.util.function.Supplier

abstract class AdjustableValue<T>(name: String, internal val def: T, widgetBuilder: (SimpleWidget) -> SimpleWidget) : Supplier<T> {
  internal val entry = widgetBuilder(Tabs.DEBUG.add(name, def)).entry

  abstract fun set(value: T)
}