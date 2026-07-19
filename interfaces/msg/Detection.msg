# One detected object. Frame is declared by the enclosing DetectionArray:
#   BODY:  x = starboard+ [m], y = forward+ [m]
#   WORLD: x = east+ [m], y = north+ [m] (relative to world origin / first RTK fix)
string label           # buoy class (5 LED states), dock feature, etc.
float32 x
float32 y
float32 radius         # estimated object radius [m]
float32 confidence     # detector confidence [0..1]

uint8 SOURCE_PERCEPTION=0
uint8 SOURCE_COMMS=1
uint8 source
