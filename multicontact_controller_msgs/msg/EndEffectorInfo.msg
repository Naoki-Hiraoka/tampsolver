Header header # header.frame_id : name of parent link

geometry_msgs/Transform transform # local transform of EndEffector in parent link coordinate
multicontact_controller_msgs/ContactInfo contact


# to be removed

string name # name of EndEffector

uint8 NOT_CARED=0
uint8 AIR=1
uint8 AIR_NEAR_CONTACT=2
uint8 TOWARD_BREAK_CONTACT=3
uint8 TOWARD_MAKE_CONTACT=4
uint8 CONTACT=5

int32 state # state of EndEffector
