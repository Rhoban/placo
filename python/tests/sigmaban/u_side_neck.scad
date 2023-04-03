% scale(1000) import("u_side_neck.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([-15.000000, -58.302042, 0]) {
    rotate([0, 0, 0.0]) {
      cube([30.000000, 65.802042, thickness]);
    }
  }
}
}
