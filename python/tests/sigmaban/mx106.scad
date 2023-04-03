% scale(1000) import("mx106.stl");

// Sketch PureShapes 46.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 23.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 46.000000;
translate([0, 0, -thickness]) {
  translate([-20.100000, -50.600000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([40.200000, 65.100000, thickness]);
    }
  }
}
}
