calibrate right and left rotation for small angle (5-15 deg)
Angle tolerance = 5-10 deg
calibrate forward movement for 10-20 cm
distance tolerance = need to test and error 

IS THE ANGLE DIFFERENCE (0 TO 360) OR (-180 TO +180)
IF  (-180 TO +180)
for angle difference calculate the time/ms and send to esp wait for AKG msg 
calculate the 3D/4 distance 
calculate the timing for forward movement and send the esp and wait fo AKG msg
if 1D/4 (might need to change to something else ) distance is less the tolerance then send PICKUP msg to esp and wait for AKG msg

do the same for target only with few changes.
