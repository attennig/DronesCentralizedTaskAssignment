\ENCODING=ISO-8859-1
\Problem name: 

Maximize
 obj1: delta_1 + delta_2
Subject To
 c_d_1_in_out_1:    - x1_1,2 - x1_1,pnt1 + x1_2,1 + x1_pnt1,1  = 0
 c_d_1_in_out_2:    x1_1,2 - x1_2,1 - x1_2,pnt1 + x1_pnt1,2  = 0
 c_d_1_in_out_pnt1: x1_1,pnt1 + x1_2,pnt1 - x1_pnt1,1 - x1_pnt1,2  = 0
 c_d_2_in_out_1:    - x2_1,2 - x2_1,pnt2 + x2_2,1 + x2_pnt2,1  = 0
 c_d_2_in_out_2:    x2_1,2 - x2_2,1 - x2_2,pnt2 + x2_pnt2,2  = 0
 c_d_2_in_out_pnt2: x2_1,pnt2 + x2_2,pnt2 - x2_pnt2,1 - x2_pnt2,2  = 0
 c_depot_1:         x1_pnt1,1 + x1_pnt1,2  = 1
 c_depot_2:         x2_pnt2,1 + x2_pnt2,2  = 1
 c_1_1follows2:     - 3 x1_1,2 - z1_1 + z1_2 >= -2
 c_1_2follows1:     - 3 x1_2,1 + z1_1 - z1_2 >= -2
 c_2_1follows2:     - 3 x2_1,2 - z2_1 + z2_2 >= -2
 c_2_2follows1:     - 3 x2_2,1 + z2_1 - z2_2 >= -2
 c_battery_1:       11 x1_1,2 + 2 x1_1,pnt1 + 11 x1_2,1 + 2 x1_2,pnt1
                    + x1_pnt1,1 + x1_pnt1,2 <= 7
 c_battery_2:       11 x2_1,2 + 2 x2_1,pnt2 + 11 x2_2,1 + 2 x2_2,pnt2
                    + x2_pnt2,1 + x2_pnt2,2 <= 7
 c_delta_1:         delta_1 - x1_1,2 - x1_1,pnt1 - x2_1,2 - x2_1,pnt2 <= 0
 c_delta_2:         delta_2 - x1_2,1 - x1_2,pnt1 - x2_2,1 - x2_2,pnt2 <= 0
Bounds
 0 <= delta_1 <= 1
 0 <= delta_2 <= 1
 0 <= x1_1,2 <= 1
 0 <= x1_1,pnt1 <= 1
 0 <= x1_2,1 <= 1
 0 <= x1_2,pnt1 <= 1
 0 <= x1_pnt1,1 <= 1
 0 <= x1_pnt1,2 <= 1
 0 <= x2_1,2 <= 1
 0 <= x2_1,pnt2 <= 1
 0 <= x2_2,1 <= 1
 0 <= x2_2,pnt2 <= 1
 0 <= x2_pnt2,1 <= 1
 0 <= x2_pnt2,2 <= 1
 1 <= z1_1 <= 2
 1 <= z1_2 <= 2
 1 <= z2_1 <= 2
 1 <= z2_2 <= 2
Binaries
 delta_1  delta_2  x1_1,2  x1_1,pnt1  x1_2,1  x1_2,pnt1  x1_pnt1,1  x1_pnt1,2 
 x2_1,2  x2_1,pnt2  x2_2,1  x2_2,pnt2  x2_pnt2,1  x2_pnt2,2 
Generals
 z1_1  z1_2  z2_1  z2_2 
End
