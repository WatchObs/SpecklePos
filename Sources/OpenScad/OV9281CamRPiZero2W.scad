$fn=180;
/*
    A box for the pizero(/w).
    Designed to allow the pizero to be completely in the box, resiliant to
    dust and pokey fingers.
    
    * Configurable connector holes
    * Either holes, for screws/bolts, or mounting pins for the pizero
    * Engineering model of a pizero, so you can check if it looks right
        - remember to turn this off before generating an STL
        
    TODO:
    * Calculate everything, remove magic numbers
    * Cut outs the gland on the lid to allow connectors right into the pizero
*/

// THINGIVERSE customizer does not understand true/flase, so use 1/0 instead

/* [parts] */
make_bottom=0;   // [0:No,1:Yes] 
make_top=0;      // [0:No,1:Yes] 
make_spacer=1;   // [0:No,1:Yes]

/* [connectors] */
sdcard_hole = 0; // [0:No,1:Yes] 
power_hole = 1;  // [0:No,1:Yes] 
hdmi_hole = 0;   // [0:No,1:Yes] 
usb_hole = 1;    // [0:No,1:Yes] 
camera_hole = 1; // [0:No,1:Yes] 
gpio_hole = 0;   // [0:No,1:Yes] 
pins = 1;        // [0:No,1:Yes] 

/* [box] */
// gap around pizero inside the box
gap = 1.0;
// wall thickness of box
shell_thickness = 1.6;
// standoffs pizero sits on in the box
standoffs = 1.5;

/* [Text on lid] */
// Add an engraved text on the top, 'help->font list' for available
add_text = 0;  // [0:No,1:Yes]
lid_text = "pizerow";
text_font="Comic Sans MS"; 

/* [pizero options] */
// show an engineering model
dummy_pizero = 0;  // [0:No,1:Yes] 
// do we need height for a header
//pz_header = 0; // not tested, problems with heights 
// allow for solder on bottom, standoffs take care of this
//pz_solder = 0;    // don't need to allow for this as we have standoffs

// the pizero engineering model with the measurements we use, includes don't work in thingiverse customizer
//include <pizerow.scad>;

//////////////////////////////
//
// Start of included pizero engineering model
//
////////////////////
/* [pizero dimensions DO NOT ALTER] */
pz_length = 65; // not including sd card protrusion
pz_width = 30;  // not including any connector protrusions
pz_pcb_thickness = 1.45; // including solder mask
pz_component_max_height = (3.1 - pz_pcb_thickness); // hdmi is max
pz_rounded_edge_offset = 3.0;
pz_botton_pin_height = 1.0;  // solder pins for gpio connector

pz_mount_hole_dia = 2.75-.2; 
pz_mount_hole_offset = 3.5; // from edge
pz_mount_hole_dia_clearance = 6; 

pz_gpio_length = 50.8; // total 
pz_gpio_width = 5;  // total
pz_gpio_x_offset = 32.5;  // from left hand edge to centre of connector
pz_gpio_y_offset = 3.5; // long edge centre form pcb edge
pz_gpio_height = (9.8 - pz_pcb_thickness); // wihtout pcb thickness

pz_sdcard_y_offset = 16.9;
pz_sdcard_length = 15.4; // sdcard present
pz_sdcard_width = 12;
pz_sdcard_protrusion = 2.3; // sdcard present
pz_sdcard_height = (2.8 - pz_pcb_thickness); 

pz_camera_y_offset = 15;
pz_camera_length = 4.43;
pz_camera_width = 17+5;
pz_camera_protrusion = 1.1; // no cable present
pz_camera_height = (2.65 - pz_pcb_thickness);

pz_hdmi_x_offset = 12.4;
pz_hdmi_length = 11.2;
pz_hdmi_width = 7.6;
pz_hdmi_protrusion = 0.5; // no cable present
pz_hdmi_height = (4.7 - pz_pcb_thickness);

pz_usb_power_x_offset = 54;
pz_usb_x_offset = 41.4;
pz_usb_length = 8;
pz_usb_width = 5.6;
pz_usb_protrusion = 1; // no cable present
pz_usb_height = (3.96 - pz_pcb_thickness);

pz_max_length = pz_length + pz_sdcard_protrusion + pz_camera_protrusion;
pz_max_width = pz_width + pz_usb_protrusion;
///////////////////////////////////
//
// End of pizero definitions
//
////////////////////

//////////////////////////////////
//
// Start of pizero functions/modules
//
////////////////////////
function pz_get_max_height(gpio_header, gpio_solder) =
    pz_pcb_thickness  + 
    (pz_botton_pin_height * (gpio_solder?1:0)) + 
    (pz_gpio_height * (gpio_header?1:0)) +
    (pz_component_max_height * (gpio_header?0:1));


module pzw(gpio_header = true, gpio_solder = true) { 

    pz_max_height = pz_get_max_height(gpio_header, gpio_solder);
    
    echo("pi max length ", pz_max_length);
    echo("pi max width  ", pz_max_width);
    echo("pi max height ", pz_max_height);

    module pzw_solid() {
        // rounded edges on pcb
        x_round = [pz_rounded_edge_offset, (pz_length - pz_rounded_edge_offset)];
        y_round= [pz_rounded_edge_offset, (pz_width - pz_rounded_edge_offset)];
        for (x = x_round, y = y_round)
            translate([x, y, 0])
            {
                $fn = 40;
                cylinder(d=(2*pz_rounded_edge_offset), h=pz_pcb_thickness);
            }  

        // pcb split into bits to conform with rounded edges
        translate([pz_rounded_edge_offset, 0, 0])
            cube([pz_length - (2 * pz_rounded_edge_offset), pz_width, pz_pcb_thickness]);
        translate([0, pz_rounded_edge_offset, 0])
            cube([pz_length, pz_width - (2 * pz_rounded_edge_offset), pz_pcb_thickness]);

        // gpio 
        if (gpio_header)
        translate([pz_gpio_x_offset-(pz_gpio_length/2), 
                  (pz_width-pz_gpio_y_offset-(pz_gpio_width/2)), 
                  pz_pcb_thickness])
            cube([pz_gpio_length, pz_gpio_width, pz_gpio_height]);

        // gpio underside solder
        if (gpio_solder)
        translate([pz_gpio_x_offset-(pz_gpio_length/2), 
                  (pz_width-pz_gpio_y_offset-(pz_gpio_width/2)), 
                  -pz_botton_pin_height])
            cube([pz_gpio_length, pz_gpio_width, pz_botton_pin_height]);
        
        // sdcard 
        translate([-pz_sdcard_protrusion, 
                  (pz_sdcard_y_offset-(pz_sdcard_width/2)), 
                  pz_pcb_thickness])
            cube([pz_sdcard_length, pz_sdcard_width, pz_sdcard_height]);

        // camera
        translate([(pz_length - pz_camera_length + pz_camera_protrusion), 
                   (pz_camera_y_offset-(pz_camera_width/2)), 
                    pz_pcb_thickness])
            cube([pz_camera_length, pz_camera_width, pz_camera_height]);
            
        // hdmi 
        translate([(pz_hdmi_x_offset - (pz_hdmi_length/2)), 
                   -pz_hdmi_protrusion, 
                    pz_pcb_thickness])
            cube([pz_hdmi_length, pz_hdmi_width, pz_hdmi_height]);
            
        // usb power 
        translate([(pz_usb_power_x_offset - (pz_usb_length/2)), 
                   -pz_usb_protrusion, 
                    pz_pcb_thickness])
            cube([pz_usb_length, pz_usb_width, pz_usb_height]);
        
        // usb 
        translate([(pz_usb_x_offset - (pz_usb_length/2)), 
                   -pz_usb_protrusion, 
                    pz_pcb_thickness])
            cube([pz_usb_length, pz_usb_width, pz_usb_height]);
    }
    
    // make 0,0,0 centre
    translate([pz_camera_protrusion+pz_camera_protrusion-pz_max_length/2, 
               pz_usb_protrusion-pz_max_width/2, 
               0])
    difference () {
        pzw_solid();

        // mounting holes
        x_holes = [pz_mount_hole_offset, (pz_length - pz_mount_hole_offset)];
        y_holes = [pz_mount_hole_offset, (pz_width - pz_mount_hole_offset)];
        for (x = x_holes, y = y_holes)
            translate([x, y, -pz_pcb_thickness])
            {
                $fn = 40;
                cylinder(d=pz_mount_hole_dia, h=10);
            }
   }
}
////////////////////////////////////////
//
// End of pizero functions/modules
//
////////////////////////////

case_inside_length = pz_max_length + 2*gap;
case_inside_width = pz_max_width + 2*gap;
case_inside_height = pz_get_max_height(true, true) + standoffs; 

case_outside_length = case_inside_length + (2*shell_thickness);
case_outside_width = case_inside_width + (2*shell_thickness);
case_outside_height = case_inside_height; //+ (2*shell_thickness);

module rounded_box(length, 
             width, 
             height, 
             rounded_edge_radius) 
{    
    
    // rounded edges
    x_round = [rounded_edge_radius, (length - rounded_edge_radius)];
    y_round= [rounded_edge_radius, (width - rounded_edge_radius)];
    for (x = x_round, y = y_round)
        translate([x, y, 0])
        {
            $fn = 40;
            cylinder(d=(2*rounded_edge_radius), h=height);
        }  

    // pcb split into bits to conform with rounded edges
    translate([rounded_edge_radius, 0, 0])
        cube([length - (2 * rounded_edge_radius), 
                width, 
                height]);
    translate([0, rounded_edge_radius, 0])
        cube([length, 
                width - (2 * rounded_edge_radius), 
                height]);    
}

module shell(inside_length, 
             inside_width, 
             inside_height, 
             thickness, 
             rounded_edge_radius) 
{
    difference () 
    {
        // outside
        translate([-thickness, -thickness, -thickness])
            rounded_box(case_outside_length, 
                case_outside_width, 
                case_outside_height,
                rounded_edge_radius);
    
        // inside, remove top by extending it through the outside
        rounded_box(inside_length, inside_width, case_outside_height+1, rounded_edge_radius);
        
        if (sdcard_hole) {
            offset = 10.4 + gap; // magic number for centre line
            translate([-case_outside_length/8, offset, standoffs+pz_pcb_thickness/2])
                cube([case_outside_length/4, pz_sdcard_width+3, 4]);
        }
        
        if (usb_hole) {
            offset = 37.9 + gap;
            translate([offset, -case_outside_width/4, standoffs-0.6])
                cube([pz_usb_length+3.5, case_outside_length/4, 7]);
        }

        if (power_hole) {
            offset = 50.5 + gap;
            translate([offset, -case_outside_width/4, standoffs-0.6])
                cube([pz_usb_length+3.5, case_outside_length/4, 7]);
        }
        
        // hole for camera
        if (camera_hole) {
            offset = 8.5 + gap;
            translate([case_outside_length/1.25, offset-3.5, (standoffs+pz_pcb_thickness)])
                cube([case_outside_length/4, pz_camera_width, 1.2+3]);
        }
        if (hdmi_hole) {
            offset = 7.3 + gap;
            translate([offset, -case_outside_width/4, standoffs+pz_pcb_thickness/2])
                cube([pz_hdmi_length+3.5, case_outside_length/4, 5]);
        }
    }
}

module bottom_shell() {
    translate([-case_inside_length/2, -case_inside_width/2, 0])
        shell(case_inside_length, 
              case_inside_width, 
              case_inside_height, 
              shell_thickness, 
              pz_rounded_edge_offset);

    // mounting standoffs and pins for pizero
    translate([pz_camera_protrusion+pz_camera_protrusion-pz_max_length/2,
                pz_usb_protrusion-pz_max_width/2, 0]) 
    {
        x_pins = [pz_mount_hole_offset, (pz_length - pz_mount_hole_offset)];
        y_pins = [pz_mount_hole_offset, (pz_width - pz_mount_hole_offset)];
        for (x = x_pins, y = y_pins)
            translate([x, y, 0])
            {
                $fn = 40;
                cylinder(d=pz_mount_hole_dia_clearance, h=standoffs);
                translate([0, 0, standoffs])
                    if (pins) 
                    {
                        // allow for some slack in the hole diameter, 0.9
                        // pins longer then pcb is thick so pcb can't slip out
                        cylinder(d=(pz_mount_hole_dia*0.9), h=3*pz_pcb_thickness);
                    }
            }
    }  
    
    // for reference we can add a dummy pizero
    if (dummy_pizero) {
        color("yellow")
            translate([0, 0, standoffs]) // add 20 for outside box
                pzw(true, true);
    }
}


module bottom() {
    difference() 
    {
        bottom_shell();
        // holes right through instead of pins
        if (!pins) 
        {
            translate([pz_camera_protrusion+pz_camera_protrusion-pz_max_length/2,
                    pz_usb_protrusion-pz_max_width/2, 0]) 
            {
                x_pins = [pz_mount_hole_offset, (pz_length - pz_mount_hole_offset)];
                y_pins = [pz_mount_hole_offset, (pz_width - pz_mount_hole_offset)];
                for (x = x_pins, y = y_pins)
                    translate([x, y, 0])
                    {
                        $fn = 40;
                        translate([0, 0, -2*shell_thickness])
                            cylinder(d=(pz_mount_hole_dia*1.1), h=2*case_inside_height);
                    }
            }
        }
    }
}
LidDepthExtend = 10;
module top_with_rim() {
    // add an extra layer on top to cover the gpio hole
    lid_thickness = (gpio_hole)?(shell_thickness):(shell_thickness+1) + LidDepthExtend;
    difference()
    {
     rounded_box(case_outside_length, case_outside_width, lid_thickness, pz_rounded_edge_offset);
     translate([(case_outside_length-case_inside_length)/2,
                (case_outside_width -case_inside_width )/2,
                -1])
     rounded_box(case_inside_length, case_inside_width, lid_thickness-shell_thickness, pz_rounded_edge_offset);
    }
    
    // need to make a rim/gland
    translate([shell_thickness, shell_thickness, -2*shell_thickness]) 
    {
        // translate to make sure that rim is part of the top
        translate([0,0,0.5]) 
        difference() 
        {
            rounded_box(case_outside_length - 2*shell_thickness, 
                         case_outside_width - 2*shell_thickness, 
                         2*shell_thickness, 
                         pz_rounded_edge_offset);
            
            // make a gland type rim by extending through the inside box
            translate([shell_thickness, shell_thickness, -shell_thickness])
                rounded_box(case_inside_length - 2*shell_thickness, 
                            case_inside_width - 2*shell_thickness, 
                            3*shell_thickness, 
                            pz_rounded_edge_offset);
        }
    }
}

module top () {
    //color("orange")
    
    translate([-case_inside_length/2-shell_thickness, 
                -case_inside_width/2-shell_thickness, 
                case_outside_height+shell_thickness]) 
        difference () 
        {
            top_with_rim();
         
            // always cut out for gpio pins
            offset_x = 9.9 + gap; // magic numbers
            offset_y = 25.4 + gap;
            translate([offset_x, offset_y, -3*shell_thickness])
                cube([pz_gpio_length+2, 
                       pz_gpio_width+2,
                       4*shell_thickness+0.3]);
            
            // a notch to make taking top off easier
/*
             translate([0, 10+gap, -0.1])
                cube([shell_thickness, 
                       15,
                       shell_thickness*0.6]);
*/            
            // text, really deep text if we have no gpio cutout
            if(add_text) 
            {
                translate([case_outside_length/2,case_outside_width*0.5,shell_thickness*0.5]) 
                    linear_extrude(5, convexity=4) 
                        text(lid_text, font=text_font, valign="center", halign="center");
            }
        }
    
    if (pins)
    {
        // add locking pins to lid
        // values iteratively found 
        translate([pz_camera_protrusion+pz_camera_protrusion-pz_max_length/2, 
                    pz_usb_protrusion-pz_max_width/2,
                    case_outside_height - 2.9*shell_thickness]) 
        {
            x_pins = [pz_mount_hole_offset, (pz_length - pz_mount_hole_offset)];
            y_pins = [pz_mount_hole_offset, (pz_width - pz_mount_hole_offset)];
            for (x = x_pins, y = y_pins)
                translate([x, y, 0])
                {
                    //translate([0, 0, -2*shell_thickness])
                    difference ()
                    {
                        $fn = 40;
                        length = 6.6; //magic
                        cylinder(d=pz_mount_hole_dia_clearance*0.9, h=length+LidDepthExtend);
                        cylinder(d=pz_mount_hole_dia*1.1, h=length+LidDepthExtend);
                    }
                }
        }
            
    }
}

/*
// something doesn't add up, measured prints do not equate
echo("inside length ", case_inside_length);
echo("inside width  ", case_inside_width);
echo("inside height  ", case_inside_height);

echo("outside length ", case_outside_length);
echo("outside width  ", case_outside_width);
echo("outside height ", case_outside_height);
*/

if (make_spacer) spacer();
if (make_top)    top();
if (make_bottom) 
{
 rotate([0,0,180]) bottom();
 bottom_additions();
}

module bottom_additions()
{
// Mounting slots
translate([0,-25,.9])
{
  difference()
  {
   cube([71,15,5], center=true);
   shift = -17;
   off = 34;
   wid = 10;
   slots = 2;   // set to 1, 2, or 3

   for (i = [0 : slots-1]) 
   {
    x = shift + off * i;
    hull() 
    {
     translate([x + wid, 0, 0 ]) cylinder(h=10, d=5.5, center=true);
     translate([x - wid, 0, 0 ]) cylinder(h=10, d=5.5, center=true);
    }
   }
  }
}
// Camera mounting
translate([-51.8,3-2,-1.6])
rotate([90,0,90]) 
{
*%rotate([90,0,0])
  color("darkseagreen")
  import("C:/Users/Andre/Desktop/Tools/CAD-CAM/STL/RPi/RPi_Camera_V2.1.stl");

// Camera backboard
cbd=10;    // Camera backboard depth
cbw=32+20; // Camera backboard width
lzd=6.25;  // Laser barrel diameter
hld=1.6;   // Camera mounting hole diameter
hlw=28.;   // OV9281 camera horizontal hole spacing
hlh=28.;   // OV9281 camera vertical   hole spacing
difference()
{
 // Camera backboard
 translate([cbw/2-18,cbd/2,-12+5])
  cube([cbw,cbd,37+10], center=true);

 // Laser side material removal (from backboard)
 translate([cbw/2+2,5,7]) cube([15,15,25],center=true);
 translate([cbw/2+2,5,-31]) cube([15,15,25],center=true);

 // Camera mounting holes
 rotate([90,0,0])
 {
  off = 12;
  translate([-hlw/2, hlh/2-off,0]) cylinder(h=30,d=hld, center=true);
  translate([ hlw/2, hlh/2-off,0]) cylinder(h=30,d=hld, center=true);
  translate([ hlw/2,-hlh/2-off,0]) cylinder(h=30,d=hld, center=true);
  translate([-hlw/2,-hlh/2-off,0]) cylinder(h=30,d=hld, center=true);
 }
 // Cable path (CSI + laser power)
 translate([-9.1-3.4,3.8-6,11.5-3]) cube([17+5,20,8]);

 // Laser tunnel
 translate([cbw/2-6,0,-12])
  rotate([90,0,-40])
   cylinder(h=cbd*6,d=lzd, center=true);

}
}
}
module spacer()
{
 //wd  = 35;
 //lng = 95;
 //ht  = 50 + 25;
 //off = 34; // see camera bracket

 wd  = 35;
 lng = 80;
 ht  = 50 + 25;
 off = 34; // see camera bracket
 

 thread = 4.5;
 translate([55,-43,-ht-10])
 rotate([0,0,90])
 difference()
 {
  // Body
  cube([wd,lng,ht]);

  // Bolt into structure
  translate([wd/2,15,ht/2+20]) cylinder(h=70,d=15, center=true);
  translate([wd/2,15,ht/2])    cylinder(h=90,d=8.5,center=true);
  // Threaded holes for camera retention
  translate([wd/2-10,lng-10,ht/2])      cylinder(h=90, d=thread, center=true);
  translate([wd/2-10,lng-10-off,ht/2])  cylinder(h=90, d=thread, center=true);
 } 
}


