% scale(1000) import("tibia_horn_side.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

rotate([0,0,-90])
translate([0, -35/2, 0])
cube([150, 35, 3]);