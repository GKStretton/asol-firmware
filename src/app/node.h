#pragma once
#include "../extras/machinepb/machine.pb.h"

/*
Note, currently only the z, pitch, and yaw axes are managed by the node system.
This is because the ring and pipette axes have no effect on collision avoidance.
Furthermore it decouples those so that they can move independently / concurrently
to node navigation. For example, go to next target ring position while collecting
dye
*/

// indexed by 1, convert number 1-n to Node.
machine_Node VialNumberToInsideNode(int number);