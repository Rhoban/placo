% scale(1000) import("x_plate.stl");

// Sketch PureShapes 6.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, -6.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 6.000000;
translate([0, 0, -thickness]) {
  translate([-66.500000, -56.500000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([133.000000, 113.000000, thickness]);
    }
  }
}
}
