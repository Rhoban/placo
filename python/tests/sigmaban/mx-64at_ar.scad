% scale(1000) import("mx-64at_ar.stl");

// Sketch PureShapes 41.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 20.5], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 41.000000;
translate([0, 0, -thickness]) {
  translate([-20.100000, -48.100000, 0]) {
    rotate([0, 0, 0.0]) {
      cube([40.200000, 61.100000, thickness]);
    }
  }
}
}
