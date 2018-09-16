int pitchMax=15, pitchMin=-pitchMax; //pitchMin is kept in case we want asymmetric bound
int rollMax=15, rollMin=-rollMax;
int yawMax=180, yawMin=-yawMax;

double pitchd, yawd, rolld;
/*is bounding yaw output necessary? 
 * These two are left here in case we use them for accurate position controll.
 *


