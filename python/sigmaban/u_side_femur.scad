% scale(1000) import("u_side_femur.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([-22.210285, -96.383519, 0]) {
    rotate([0, 0, 0.0]) {
      cube([44.420570, 96.383519, thickness]);
    }
  }
}
}
