% scale(1000) import("u_side_small.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([30.000000, 47.500000, 0]) {
    rotate([0, 0, 180.0]) {
      cube([30.000000, 47.500000, thickness]);
    }
  }
}
}
