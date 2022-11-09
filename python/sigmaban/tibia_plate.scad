% scale(1000) import("tibia_plate.stl");

// Sketch PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([14.000000, 16.098550, 0]) {
    rotate([0, 0, -180.0]) {
      cube([40.426218, 170.098550, thickness]);
    }
  }
}
}
