# Exercises every .msg grammar feature the generator supports: constants, scalar defaults,
# bounded strings, fixed/unbounded/bounded arrays, octet fast paths, and nested messages
# resolved from the embedded common set.

uint8 SMALL=7
string GREETING="hi there"

bool flag true
uint8 tiny
float64 wide
string label "default label"
string<=16 bounded_label
byte[4] raw
uint8[] blob
uint8[<=100] bounded_blob
int16[3] triplet
float64[] samples
string[] names
std_msgs/Header header
std_msgs/String note
geometry_msgs/Twist twist
nav_msgs/Odometry odom
builtin_interfaces/Duration timeout
