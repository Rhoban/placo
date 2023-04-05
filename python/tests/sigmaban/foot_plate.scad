% scale(1000) import("foot_plate.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([46.000000, 78.800000, 0]) {
    rotate([0, 0, 180.0]) {
      cube([92.000000, 157.600000, thickness]);
    }
  }
}
}
