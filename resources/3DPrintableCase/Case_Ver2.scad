
/*//////////////////////////////////////////////////////////////////
              -    FB Aka Heartman/Hearty 2016     -                   
              -   http://heartygfx.blogspot.com    -                  
              -       OpenScad Parametric Box      -                     
              -         CC BY-NC 3.0 License       -                      
////////////////////////////////////////////////////////////////////                                                                                                             
12/02/2016 - Fixed minor bug 
28/02/2016 - Added holes ventilation option                    
09/03/2016 - Added PCB feet support, fixed the shell artefact on export mode. 

*/////////////////////////// - Info - //////////////////////////////

// All coordinates are starting as integrated circuit pins.
// From the top view :

//   CoordD           <---       CoordC
//                                 ^
//                                 ^
//                                 ^
//   CoordA           --->       CoordB


////////////////////////////////////////////////////////////////////


////////// - Paramètres de la boite - Box parameters - /////////////

/* [Box dimensions] */
// - Longueur - Length  
  Length        = 140;       
// - Largeur - Width
  Width         = 140;                     
// - Hauteur - Height  
  Height        = 50;  
// - Epaisseur - Wall thickness  
  Thick         = 3;//[2:5]  
  
/* [Box options] */
// - Diamètre Coin arrondi - Filet diameter  
  Filet         = 4;//[0.1:12] 
// - lissage de l'arrondi - Filet smoothness  
  Resolution    = 50;//[1:100] 
// - Tolérance - Tolerance (Panel/rails gap)
  m             = 0.9;
// Pieds PCB - PCB feet (x4) 
  PCBFeet       = 1;// [0:No, 1:Yes]
  PCBFeet2       = 1;// [0:No, 1:Yes]
// - Decorations to ventilation holes
  Vent          = 1;// [0:No, 1:Yes]
// - Decoration-Holes width (in mm)
  Vent_width    = 2;   


  
/* [PCB_Feet] */
//All dimensions are from the center foot axis

// - Coin bas gauche - Low left corner X position
PCBPosX         = 5;
// - Coin bas gauche - Low left corner Y position
PCBPosY         = 3;
// - Longueur PCB - PCB Length
PCBLength       = 85.5;
// - Largeur PCB - PCB Width
PCBWidth        = 65.5;
// - Heuteur pied - Feet height
FootHeight      = 10;
// - Diamètre pied - Foot diameter
FootDia         = 8;
// - Diamètre trou - Hole diameter
FootHole        = 3;  
  
PCBPosX2         = 5;
// - Coin bas gauche - Low left corner Y position
PCBPosY2         = 79;
// - Longueur PCB - PCB Length
PCBLength2       = 95.5;
// - Largeur PCB - PCB Width
PCBWidth2        = 33;
// - Heuteur pied - Feet height
FootHeight2      = 10;
// - Diamètre pied - Foot diameter
FootDia2         = 8;
// - Diamètre trou - Hole diameter
FootHole2        = 3; 

/* [STL element to export] */
//Coque haut - Top shell
TShell          = 1;// [0:No, 1:Yes]
//Coque bas- Bottom shell
BShell          = 1;// [0:No, 1:Yes]
//Panneau avant - Front panel
FPanL           = 1;// [0:No, 1:Yes]
//Panneau arrière - Back panel  
BPanL           = 1;// [0:No, 1:Yes]


  
/* [Hidden] */
// - Couleur coque - Shell color  
Couleur1        = "Orange";       
// - Couleur panneaux - Panels color    
Couleur2        = "OrangeRed";    
// Thick X 2 - making decorations thicker if it is a vent to make sure they go through shell
Dec_Thick       = Vent ? Thick*2 : Thick; 
// - Depth decoration
Dec_size        = Vent ? Thick*2 : 0.8;





/////////// - Boitier générique bord arrondis - Generic rounded box - //////////

module RoundBox($a=Length, $b=Width, $c=Height){// Cube bords arrondis
                    $fn=Resolution;            
                    translate([0,Filet,Filet]){  
                    minkowski (){                                              
                        cube ([$a-(Length/2),$b-(2*Filet),$c-(2*Filet)], center = false);
                        rotate([0,90,0]){    
                        cylinder(r=Filet,h=Length/2, center = false);
                            } 
                        }
                    }
                }// End of RoundBox Module

      
////////////////////////////////// - Module Coque/Shell - //////////////////////////////////         

module Coque(){//Coque - Shell  
    Thick = Thick*2;  
    difference(){    
        difference(){//sides decoration
            union(){    
                     difference() {//soustraction de la forme centrale - Substraction Fileted box
                      
                        difference(){//soustraction cube median - Median cube slicer
                            union() {//union               
                            difference(){//Coque    
                                RoundBox();
                                translate([Thick/2,Thick/2,Thick/2]){     
                                        RoundBox($a=Length-Thick, $b=Width-Thick, $c=Height-Thick);
                                        }
                                        }//Fin diff Coque                            
                                difference(){//largeur Rails        
                                     translate([Thick+m,Thick/2,Thick/2]){// Rails
                                          RoundBox($a=Length-((2*Thick)+(2*m)), $b=Width-Thick, $c=Height-(Thick*2));
                                                          }//fin Rails
                                     translate([((Thick+m/2)*1.55),Thick/2,Thick/2+0.1]){ // +0.1 added to avoid the artefact
                                          RoundBox($a=Length-((Thick*3)+2*m), $b=Width-Thick, $c=Height-Thick);
                                                    }           
                                                }//Fin largeur Rails
                                    }//Fin union                                   
                               translate([-Thick,-Thick,Height/2]){// Cube à soustraire
                                    cube ([Length+100, Width+100, Height], center=false);
                                            }                                            
                                      }//fin soustraction cube median - End Median cube slicer
                               translate([-Thick/2,Thick,Thick]){// Forme de soustraction centrale 
                                    RoundBox($a=Length+Thick, $b=Width-Thick*2, $c=Height-Thick);       
                                    }                          
                                }                                          


                difference(){// wall fixation box legs
                    union(){
                        translate([3*Thick +5,Thick,Height/2]){
                            rotate([90,0,0]){
                                    $fn=6;
                                    cylinder(d=16,Thick/2);
                                    }   
                            }
                            
                       translate([Length-((3*Thick)+5),Thick,Height/2]){
                            rotate([90,0,0]){
                                    $fn=6;
                                    cylinder(d=16,Thick/2);
                                    }   
                            }

                        }
                            translate([4,Thick+Filet,Height/2-57]){   
                             rotate([45,0,0]){
                                   cube([Length,40,40]);    
                                  }
                           }
                           translate([0,-(Thick*1.46),Height/2]){
                                cube([Length,Thick*2,10]);
                           }
                    } //Fin fixation box legs
            }

        union(){// outbox sides decorations
            
            for(i=[0:Thick:Length/4]){

                // Ventilation holes part code submitted by Ettie - Thanks ;) 
                    translate([10+i,-Dec_Thick+Dec_size,1]){
                    cube([Vent_width,Dec_Thick,Height/4]);
                    }
                    translate([(Length-10) - i,-Dec_Thick+Dec_size,1]){
                    cube([Vent_width,Dec_Thick,Height/4]);
                    }
                    translate([(Length-10) - i,Width-Dec_size,1]){
                    cube([Vent_width,Dec_Thick,Height/4]);
                    }
                    translate([10+i,Width-Dec_size,1]){
                    cube([Vent_width,Dec_Thick,Height/4]);
                    }
  
                
                    }// fin de for
               // }
                }//fin union decoration
            }//fin difference decoration


            union(){ //sides holes
                $fn=50;
                translate([3*Thick+5,20,Height/2+4]){
                    rotate([90,0,0]){
                    cylinder(d=2,20);
                    }
                }
                translate([Length-((3*Thick)+5),20,Height/2+4]){
                    rotate([90,0,0]){
                    cylinder(d=2,20);
                    }
                }
                translate([3*Thick+5,Width+5,Height/2-4]){
                    rotate([90,0,0]){
                    cylinder(d=2,20);
                    }
                }
                translate([Length-((3*Thick)+5),Width+5,Height/2-4]){
                    rotate([90,0,0]){
                    cylinder(d=2,20);
                    }
                }
            }//fin de sides holes

        }//fin de difference holes
}// fin coque 

////////////////////////////// - Experiment - ///////////////////////////////////////////





/////////////////////// - Foot with base filet - /////////////////////////////
module foot(FootDia,FootHole,FootHeight){
    Filet=2;
    color(Couleur1)   
    translate([0,0,Filet-1.5])
    difference(){
    
    difference(){
            //translate ([0,0,-Thick]){
                cylinder(d=FootDia+Filet,FootHeight-Thick, $fn=100);
                        //}
                    rotate_extrude($fn=100){
                            translate([(FootDia+Filet*2)/2,Filet,0]){
                                    minkowski(){
                                            square(10);
                                            circle(Filet, $fn=100);
                                        }
                                 }
                           }
                   }
            cylinder(d=FootHole,FootHeight+1, $fn=100);
               }          
}// Fin module foot
  
module Feet(){     
//////////////////// - PCB only visible in the preview mode - /////////////////////    
    translate([3*Thick+2,Thick+5,FootHeight+(Thick/2)-0.5]){
    
    %square ([PCBLength+10,PCBWidth+10]);
       translate([PCBLength/2,PCBWidth/2,0.5]){ 
        color("Olive")
        %text("PCB", halign="center", valign="center", font="Arial black");
       }
    } // Fin PCB 
  
    
////////////////////////////// - 4 Feet - //////////////////////////////////////////     
    translate([3*Thick+7,Thick+10,Thick/2]){
        foot(FootDia,FootHole,FootHeight);
    }
    translate([(3*Thick)+PCBLength+7,Thick+10,Thick/2]){
        foot(FootDia,FootHole,FootHeight);
        }
    translate([(3*Thick)+PCBLength+7,(Thick)+PCBWidth+10,Thick/2]){
        foot(FootDia,FootHole,FootHeight);
        }        
    translate([3*Thick+7,(Thick)+PCBWidth+10,Thick/2]){
        foot(FootDia,FootHole,FootHeight);
    }   

} // Fin du module Feet
 

module Feet2(){     
//////////////////// - PCB only visible in the preview mode - /////////////////////    
    translate([3*Thick+2,Thick+5,FootHeight2+(Thick/2)-0.5]){
    
    %square ([PCBLength2+10,PCBWidth2+10]);
       translate([PCBLength2/2,PCBWidth2/2,0.5]){ 
        color("Olive")
        %text("PCB2", halign="center", valign="center", font="Arial black");
       }
    } // Fin PCB 
  
    
////////////////////////////// - 4 Feet - //////////////////////////////////////////     
    translate([3*Thick+7,Thick+10,Thick/2]){
        foot(FootDia2,FootHole2,FootHeight2);
    }
    translate([(3*Thick)+PCBLength2+7,Thick+10,Thick/2]){
        foot(FootDia2,FootHole2,FootHeight2);
        }
    translate([(3*Thick)+PCBLength2+7,(Thick)+PCBWidth2+10,Thick/2]){
        foot(FootDia2,FootHole2,FootHeight2);
        }        
    translate([3*Thick+7,(Thick)+PCBWidth2+10,Thick/2]){
        foot(FootDia2,FootHole2,FootHeight2);
    }   

} // Fin du module Feet
 
 ////////////////////////////////////////////////////////////////////////
////////////////////// <- Holes Panel Manager -> ///////////////////////
////////////////////////////////////////////////////////////////////////

//                           <- Panel ->  
module Panel(Length,Width,Thick,Filet){
    scale([0.5,1,1])
    minkowski(){
            cube([Thick,Width-(Thick*2+Filet*2+m),Height-(Thick*2+Filet*2+m)]);
            translate([0,Filet,Filet])
            rotate([0,90,0])
            cylinder(r=Filet,h=Thick, $fn=100);
      }
}



//                          <- Circle hole -> 
// Cx=Cylinder X position | Cy=Cylinder Y position | Cdia= Cylinder dia | Cheight=Cyl height
module CylinderHole(OnOff,Cx,Cy,Cdia){
    if(OnOff==1)
    translate([Cx,Cy,-1])
        cylinder(d=Cdia,10, $fn=50);
}
//                          <- Square hole ->  
// Sx=Square X position | Sy=Square Y position | Sl= Square Length | Sw=Square Width | Filet = Round corner
module SquareHole(OnOff,Sx,Sy,Sl,Sw,Filet){
    if(OnOff==1)
     minkowski(){
        translate([Sx+Filet/2,Sy+Filet/2,-1])
            cube([Sl-Filet,Sw-Filet,10]);
            cylinder(d=Filet,h=10, $fn=100);
       }
}

module CylinderAndSquareHole(OnOff,Cx,Cy,Cdia,Sx,Sy,Sl,Sw){
    if(OnOff==1) {
        translate([Cx,Cy,-1])
            cylinder(d=Cdia,10, $fn=50);

        translate([Sx,Sy,-1])
            cube([Sl,Sw,10]);
            }
}

// dM = Diameter, hG = Height, dP = Depth
module PowerJack0(dM, hG, dP, sX, sY) {
        union(){
            translate([sX,sY,-1]) cylinder(d=dM, h=hG, center=true, $fn=32);
            translate([sX,sY-3.55,-1]) cube([dM,dP,hG],center=true);
            }
        }
        
//                      <- Linear text panel -> 
module LText(OnOff,Tx,Ty,Font,Size,Content){
    if(OnOff==1)
    translate([Tx,Ty,Thick])
    linear_extrude(height = 1){
    text(Content, size=Size, font=Font);
    }
}
//                     <- Circular text panel->  
module CText(OnOff,Tx,Ty,Font,Size,TxtRadius,Angl,Turn,Content){ 
      if(OnOff==1) {
      Angle = -Angl / len(Content);
      translate([Tx,Ty,Thick+.5])
          for (i= [0:len(Content)-1] ){   
              rotate([0,0,i*Angle+90+Turn])
              translate([0,TxtRadius,0]) {
                linear_extrude(height = 0.5){
                text(Content[i], font = Font, size = Size,  valign ="baseline", halign ="center");
                    }
                }   
             }
      }
}
////////////////////// <- New module Panel -> //////////////////////
module FPanL(){
    difference(){
        color(Couleur2)
        Panel(Length,Width,Thick,Filet);
    
 
    rotate([90,0,90]){
        color(Couleur2){
//                     <- Cutting shapes from here ->  
        // Panel
        SquareHole  (1,87,9.5,35,23,3); //(On/Off, Xpos,Ypos,Length,Width,Filet)
        // Panel holes
        // LB
        CylinderHole(1,89,6.5,3.5);
        // RB
        CylinderHole(1,119.4,6.5,3.5);
        // LT
        CylinderHole(1,89,35.18,3.5);
        // RT
        CylinderHole(1,119.4,35.18,3.5);
        //SquareHole  (1,40,20,15,10,1);
        //SquareHole  (1,60,20,15,10,1); 
        // Led 0
        //CylinderHole(1,43,26,3.1);//(On/Off, Xpos, Ypos, Diameter)
        SquareHole  (1,33.5,20,5.5,2.5,1);
        // Led 1
        //CylinderHole(1,63,26,3.1);
        SquareHole  (1,51.5,20,5.5,2.5,1);
        // Led 2
        //CylinderHole(1,83,26,3.1);
        SquareHole  (1,69.5,20,5.5,2.5,1);
        //SquareHole  (1,20,50,80,30,3);
        // Button
        CylinderHole(1,18,21,12.5);
        //SquareHole  (1,120,20,30,60,3);
//                            <- To here -> 
           }
       }


    color(Couleur1){
        translate ([-.5,0,0])
        rotate([90,0,90]){
//                      <- Adding text from here ->          
        //LText(1,20,83,"Arial Black",4,"Digital Screen");//(On/Off, Xpos, Ypos, "Font", Size, "Text")
        LText(1,29,32,"Arial Black",3.5,"GNSS Time Server");
        LText(1,33.5,10,"Icons",4,"");
        LText(1,52,10,"Icons",4,"");
        LText(1,69.5,10,"Icons",4,"");
        //LText(1,20,11,"Arial Black",6,"  1     2      3");
        //CText(1,93,29,"Arial Black",4,10,180,0,"1 . 2 . 3 . 4 . 5 . 6");//(On/Off, Xpos, Ypos, "Font", Size, Diameter, Arc(Deg), Starting Angle(Deg),"Text")
//                            <- To here ->
            }
      }
      
       }

}

module BPanL(){
    difference(){
        color(Couleur2)
        Panel(Length,Width,Thick,Filet);
    
 
    rotate([90,0,90]){
        color(Couleur2){
//                     <- Cutting shapes from here ->  
        // WiFi
        CylinderHole(1,96,21,6.5);
        // Power
        //CylinderHole(1,66,21,11.5);
        // GNSS
        CylinderHole(1,116,21,6.5);
        // Switch
        CylinderHole(1,42,21,12.5);
        // CylinderAndSquareHole(OnOff,Cx,Cy,Cdia,Sx,Sy,Sl,Sw)
        // Cx=Cylinder X position | Cy=Cylinder Y position | Cdia= Cylinder dia | Sx=Square X position | Sy=Square Y position | Sl= Square Length | Sw=Square Width
        //CylinderAndSquareHole(1,40,21,9.5,35.25,11.5,9.5,9.5);
        // New power
        CylinderHole(1,20,28.5,3);
        CylinderHole(1,20,13,3);
        PowerJack0(9.5, 11, 7.4, 20, 22);
        //SquareHole  (1,120,20,30,60,3);
//                            <- To here -> 
           }
       }

    color(Couleur1){
        translate ([-.5,0,0])
        rotate([90,0,90]){
//                      <- Adding text from here ->          
        //LText(1,20,83,"Arial Black",4,"Digital Screen");//(On/Off, Xpos, Ypos, "Font", Size, "Text")
        //LText(1,120,83,"Arial Black",4,"Level");
        //LText(1,20,11,"Arial Black",6,"  1     2      3");
        //CText(1,93,29,"Arial Black",4,10,180,0,"1 . 2 . 3 . 4 . 5 . 6");//(On/Off, Xpos, Ypos, "Font", Size, Diameter, Arc(Deg), Starting Angle(Deg),"Text")
        // WiFi
        LText(1,114,8,"Icons",4,"");
        // Power
        LText(1,18,4,"Icons",4,"");
        // GNSS
        LText(1,94,8,"Icons",4,"");
        // Switch
        LText(1,40,4,"Icons",4,"");
        LText(1,7,33,"Arial Black",3.5,"https://github.com/Montecri/GNSSTimeServer");
//                            <- To here ->
            }
      }
       }

}

module Logo() {
translate([-138.25,-138.1,-1]) {
// Para ver furado através:
//translate([-138.25,-138.1,-10]) {
scale([0.45, 0.45, 1]) {
  linear_extrude(height = 4, convexity = 10) {
  // Para ver furado através:
  //linear_extrude(height = 40, convexity = 10) {
        import("qr-code6.dxf");
            }
        }
        }
  }

/////////////////////////// <- Main part -> /////////////////////////

if(TShell==1)
// Coque haut - Top Shell
        color( Couleur1,1){
            translate([0,Width,Height+0.2]){
                rotate([0,180,180]){
                    difference() {
                        Coque();
                        rotate([0,180,90])
                            Logo();
                            }
                        }
                }
        }

if(BShell==1)
// Coque bas - Bottom shell
        color(Couleur1){ 
        Coque();
        }

// Pied support PCB - PCB feet
if (PCBFeet==1)
// Feet
        translate([PCBPosX,PCBPosY,0]){ 
        Feet();
        }

if (PCBFeet2==1)
// Feet2
        translate([PCBPosX2,PCBPosY2,0]){ 
        Feet2();
        }
        
// Panneau avant - Front panel  <<<<<< Text and holes only on this one.
//rotate([0,-90,-90]) 
if(FPanL==1)
        translate([Length-(Thick*2+m/2),Thick+m/2,Thick+m/2])
        FPanL();

//Panneau arrière - Back panel
if(BPanL==1)
        //color(Couleur2)
        translate([Thick+m/2,Thick+m/2,Thick+m/2])
        //Panel(Length,Width,Thick,Filet);
        BPanL();