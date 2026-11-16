$fn = 96;

// ---------- COTES PLAN ----------
L = 127.76;
W = 85.48;
H_total = 19.0;
H_plate = 13.2;

lid_L = 127.4;
lid_W = 85.1;
lid_H = 8.8;
lid_z = H_total - lid_H;

cols = 8;
rows = 6;
pitch = 13;

x_first = 18.38;
y_first_top_plate = 10.24;
y_first_top_lid = 10.05;

well_OD = 11.95;
well_ID = 10.6;

R = 3.2;
cut = 13.0;

// ---------- OUTILS ----------
module rounded2d(l,w,r){
    offset(r=r) offset(delta=-r) square([l,w]);
}

module top_chamfered2d(l,w,r,cut){
    difference(){
        rounded2d(l,w,r);

        // chanfrein UNIQUEMENT côté gauche haut/bas
        polygon([[0,w-cut],[0,w+5],[cut+5,w+5]]);
        polygon([[0,cut],[0,-5],[cut+5,-5]]);
    }
}

module box_round(l,w,h,r){
    linear_extrude(h) rounded2d(l,w,r);
}

module box_chamfer_top(l,w,h,r,cut){
    linear_extrude(h) top_chamfered2d(l,w,r,cut);
}

module hollow_round(l,w,h,t,r){
    difference(){
        box_round(l,w,h,r);
        translate([t,t,-0.1])
            box_round(l-2*t,w-2*t,h+0.2,max(r-t,0.4));
    }
}

module hollow_chamfer(l,w,h,t,r,cut){
    difference(){
        box_chamfer_top(l,w,h,r,cut);
        translate([t,t,-0.1])
            box_chamfer_top(l-2*t,w-2*t,h+0.2,max(r-t,0.4),max(cut-t,0));
    }
}

// ---------- BASE : PAS DE CHANFREIN JUSQU’EN BAS ----------
module plate_base(){
    color([0.92,0.92,0.92,0.32])
    union(){
        // bas rectangulaire arrondi
        hollow_round(L,W,H_plate,1.4,R);

        // fond intérieur
        translate([4.8,4.8,2.0])
            box_round(L-9.6,W-9.6,1.2,2.2);

        // partie haute avec coins coupés, seulement au-dessus
        translate([0,0,H_plate-2.2])
            hollow_chamfer(L,W,2.2,2.0,R,cut);

        // marche extérieure basse
        translate([0,0,0])
            hollow_round(L,W,3.0,1.8,R);
    }
}

// ---------- PUITS SIMPLIFIÉS ----------
module well(x,y){
    color([1,1,1,0.36])
    translate([x,y,3.0])
    difference(){
        cylinder(h=13.5,d=well_OD);
        translate([0,0,0.4])
            cylinder(h=14,d=well_ID);
    }

    color([1,1,1,0.50])
    translate([x,y,16.2])
    difference(){
        cylinder(h=0.7,d=well_OD+0.7);
        translate([0,0,-0.1])
            cylinder(h=1.0,d=well_ID);
    }
}

module wells(){
    for(r=[0:rows-1])
        for(c=[0:cols-1]){
            x = x_first + c*pitch;
            y = W - y_first_top_plate - r*pitch;
            well(x,y);
        }
}

// ---------- NERVURES BAS / CÔTÉS ----------
module side_ribs(){
    color([0.85,0.85,0.85,0.38]){
        for(x=[18:3:L-18]){
            translate([x,0.2,2.2]) cube([0.55,1.1,8.8]);
            translate([x,W-1.3,2.2]) cube([0.55,1.1,8.8]);
        }

        for(y=[18:3:W-18]){
            translate([0.2,y,2.2]) cube([1.1,0.55,8.8]);
            translate([L-1.3,y,2.2]) cube([1.1,0.55,8.8]);
        }
    }
}

// ---------- COUVERCLE FERMÉ ----------
module lid_closed(){
    dx = (L-lid_L)/2;
    dy = (W-lid_W)/2;

    translate([dx,dy,lid_z])
    color([1,1,1,0.27])
    union(){
        // jupe du couvercle avec coins coupés
        hollow_chamfer(lid_L,lid_W,lid_H,1.15,R,cut);

        // surface supérieure
        translate([0,0,lid_H-1.1])
            box_chamfer_top(lid_L,lid_W,1.1,R,cut);

        // double rebord supérieur
        translate([2.2,2.2,lid_H-0.35])
        difference(){
            box_chamfer_top(lid_L-4.4,lid_W-4.4,0.8,2.6,cut-2.2);
            translate([2.8,2.8,-0.1])
                box_chamfer_top(lid_L-10,lid_W-10,1.0,1.8,cut-5);
        }

        // empreintes circulaires visibles sur le couvercle
        for(r=[0:rows-1])
            for(c=[0:cols-1]){
                x = 18.2 + c*pitch;
                y = lid_W - y_first_top_lid - r*pitch;
                translate([x,y,lid_H-0.15])
                difference(){
                    cylinder(h=0.35,d=11.5);
                    translate([0,0,-0.1]) cylinder(h=0.6,d=10.4);
                }
            }
    }
}

// ---------- ASSEMBLAGE ----------
plate_base();
side_ribs();
wells();
lid_closed();