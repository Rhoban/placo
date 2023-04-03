% scale(1000) import("leg.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:

for (y=[-16,16])
translate([0,y,-35])
cube([20, 2, 90], center=true);
// cylinder(r=10, h=10, center=true);

translate([0,0,-90])
sphere(15);
