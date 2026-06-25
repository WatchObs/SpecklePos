$fn=20;
/*
  Adhesive Raspberry Pi Camera v2.1 mount (OpenSCAD)
  Author: Brian Khuu (2019)
  
  This is a pressfit design. No support required. Just print and push the camera PCB to the mounting holes.
  
  Apply adhesive strip
  
  Default value assumes: Command Strip dimentions is typically 50mmx15mm

*/

/* [Camera]  */

// Camera Tilt Angle
picam_tilt_angle = 50;

// Offset Length
picam_mount_offset = 40;

/* [Mounting Strip Base]  */

// length of mounting strip base
sx = 50 ; 

// widith of mounting strip base
sy = 15 ;

// height of mounting strip base
sh = 2  ;

module rpi_camera_model() {
  // ==============================================
  // Model of Raspberry Pi Camera v2.1
  // J.Beale 14-May-2016
  // ==============================================
  
  xs = 25.0;  // width of PCB
  ys = 23.85;  // height of PCB
  zs = 0.95;  // thickness of bare PCB, not including soldermask
  smt = 0.05;  // soldermask thickness (on each side)

  ID1 = 2.20; // ID of mounting through-holes
  OD1 = 4.7;  // silkscreen clearance around hole

  mhxc = 21.0;  // X center-to-center separation of mounting holes
  mhyc = 12.5; // Y center-to-center separation of mtng. holes
  mhox = 2.0;  // X mounting hole corner offset
  mhoy = 2.0;  // Y mounting hole corner offset
  hh = zs*2; // depth/height of mounting hole (subtractive cylinder)

  smx = 8.45;  // camera sensor module, base part
  smy = 8.45;
  smz = 2.3;
  smbod = 7.25;  // OD of lens holder barrel
  smbz = 1.7; // z-height of lens holder barrel
  smcod = 5.6;  // OD of lens cell 
  smcz = 0.6;  // protrusion of cell above holder barrel
  smcid = 1.55;  // optical aperture: opening for lens

  smfx = 8.9; // flex connector width
  smfy = 4.5; // flex connector 
  smfz = 1.52; // flex connector
  smfox = 9.45; // flex edge X offset
  smfoy = 2.6; // flex edge Y offset
  smfcx = 7.2; // flex cable width

  fcy = 5.8; // 15-way flex connector
  fcx = 21.0; // 15-way flex connector
  fcz = 3.77-1.15; // 15-way flex connector height  
  fcyoff = 0.3; // connector offset from PCB edge

  ffx = 16.05;  // 15-way flex cable width
  ffy = 30;  // length of flex cable stub
  ffz = 0.38;  // thickness of ff cable at stiffener
  ffoz = 1.1; // offset of ff cable below PCB

  KOx = 22.15; // keepout width- full
  KOxa = 15.1; // keepout width- inner block
  KOz = 1.35; // bottom side keepout z-height
  KOyo = 1.15; // keepout edge offset

  fn=40;  // facets on cylinder
  eps=0.03; // small number

  module pcb() {
  difference() {
   union() {
       translate([0,0,0]) color("green") 
          cube([xs,ys,smt]); // soldermask (bottom)
       translate([0,0,smt]) color("yellow") 
          cube([xs,ys,zs]); // natural PCB color
       translate([0,0,zs]) color("green") 
          cube([xs,ys,smt]); // soldermask (top)
   }
   translate([mhox,mhoy,zs-(smt+eps)]) {  // mask clearance: top
       cylinder(d=OD1,h=zs,$fn=fn);
       translate([mhxc,0,0]) cylinder(d=OD1,h=zs,$fn=fn);
       translate([mhxc,mhyc,0]) cylinder(d=OD1,h=zs,$fn=fn);
       translate([0,mhyc,0]) cylinder(d=OD1,h=zs,$fn=fn);
   }
   translate([mhox,mhoy,-eps]) {  // mask clearance: bottom
       cylinder(d=OD1,h=smt,$fn=fn);
       translate([mhxc,0,0]) cylinder(d=OD1,h=smt,$fn=fn);
       translate([mhxc,mhyc,0]) cylinder(d=OD1,h=smt,$fn=fn);
       translate([0,mhyc,0]) cylinder(d=OD1,h=smt,$fn=fn);
   }
   translate([mhox,mhoy,-zs*5]) {  // through holes
    cylinder(d=ID1,h=zs*10,$fn=fn);     
    translate([mhxc,0,0])  cylinder(d=ID1,h=zs*10,$fn=fn);
    translate([mhxc,mhyc,0])  cylinder(d=ID1,h=zs*10,$fn=fn);
    translate([0,mhyc,0])  cylinder(d=ID1,h=zs*10,$fn=fn);
    }
   }
  }
  
  module sensor() { // Camera sensor module
   color([.25,.25,.25]) translate([-smx/2, -smy/2, 0]) cube([smx,smy,smz]);
   color([.3,.3,.3]) cylinder(d=smbod, h=smbz + smz, $fn=fn);
   difference() {
    color([.2,.2,.2]) cylinder(d=smcod, h=smbz + smz + smcz, $fn=fn);
    translate([0,0,smcz]) cylinder(d=smcid, h=smbz + smz + smcz, $fn=fn);
   }
  }

  module flex() { // micro-flex attached to sensor
   color([0.2,0.20,0.18]) cube([smfx,smfy,smfz]); // u-flex connector top
   translate([0,smfy,0]) color([0.33,0.33,0.23]) cube([smfcx,smfy,smfz-.1]);
  }

  module fcon() {  // micro-flex connector to PCB
      color([0.6,0.20,0.18]) cube([fcx,fcy,fcz]);
  }

  module ff15() {  // 15-conductor flat flex cable
      color([.7,.7,.7]) cube([ffx,ffy,ffz]);
  }
  
  translate([xs/2 ,mhyc+mhoy,zs]) sensor();
  translate([smfox,smfoy,zs]) flex(); // micro-flex on sensor
  pcb();
}

module cam_holder(thickness, rotate, hull_base)
{
  fn=30;
  pressfit_h=5;
  
  // Simulated Camera
  %rotate([rotate,0,0])translate([0 ,0,thickness+1]) rpi_camera_model();
  
  xs = 25.0;  // width of PCB
  ys = 23.85-7;  // height of PCB
  zs = 0.95;  // thickness of bare PCB, not including soldermask

  mhxc = 21.0;  // X center-to-center separation of mounting holes
  mhyc = 12.5; // Y center-to-center separation of mtng. holes
  mhox = 2.0;  // X mounting hole corner offset
  mhoy = 2.0;  // Y mounting hole corner offset
  ID1 = 2.20; // ID of mounting through-holes
  
  if(hull_base>0)
  hull()
  {
    rotate([rotate,0,0])
      cube([xs,ys,0.1]);
    cube([xs,hull_base,0.1]);
  }

  rotate([rotate,0,0])
  {
   cube([xs,ys,thickness]);
   translate([mhox,mhoy,0]) 
   {  // through holes
      translate([0,0,0])        cylinder(d1=ID1*1.1,d2=ID1*0.8,h=pressfit_h,$fn=fn);     
      translate([mhxc,0,0])     cylinder(d1=ID1*1.1,d2=ID1*0.8,h=pressfit_h,$fn=fn);
      translate([mhxc,mhyc,0])  cylinder(d1=ID1*1.1,d2=ID1*0.8,h=pressfit_h,$fn=fn);
      translate([0,mhyc,0])     cylinder(d1=ID1*1.1,d2=ID1*0.8,h=pressfit_h,$fn=fn);
   }
 }
}

union()
{
  hull()
  {
    translate([0,-sx/2,0])
      cube([1,sx,sy]);
    translate([0,0,5/2])
      rotate([0,90,0])
      cylinder(r=5/2,h=3);
  }

  translate([0,0,5/2])
    rotate([0,90,0])
      cylinder(r=5/2,h=picam_mount_offset);
  
  translate([picam_mount_offset,-5,0])
    cam_holder(2, picam_tilt_angle, 7);
}