% scale(1000) import("u_base_motors_blocs.stl");

// Sketch PureShapes 6.25
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 6.250000;
translate([0, 0, -thickness]) {
  translate([0.000000, -30.000000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([52.340000, 30.000000, thickness]);
    }
  }
}
}
