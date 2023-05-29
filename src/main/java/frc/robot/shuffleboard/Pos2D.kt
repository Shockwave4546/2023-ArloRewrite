package frc.robot.shuffleboard

data class Pos2D(val x: Int, val y: Int) {
  companion object {
    val NO_POSITION = Pos2D(-1, -1)
  }
}