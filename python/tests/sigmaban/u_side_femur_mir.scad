% scale(1000) import("u_side_femur_mir.stl");

// Sketch mir_PureShapes 3.0
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 46.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([22.244202, 96.426435, 0]) {
    rotate([0, 0, 180.0]) {
      cube([44.488404, 96.426435, thickness]);
    }
  }
}
}
