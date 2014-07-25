uint16      user_number
uint16[25]  joint_type
uint16[25]  tracking_state
# Position
float32[25] position_x
float32[25] position_y
float32[25] position_z
# Orientation
float32[25] orientation_x
float32[25] orientation_y
float32[25] orientation_z
float32[25] orientation_w
# Hand states
uint16      l_hand_state
uint16      r_hand_state

# Joint Types
uint16  SpineBase=0
uint16  SpineMid=1
uint16  Neck=2
uint16  Head=3
uint16  ShoulderLeft=4
uint16  ElbowLeft=5
uint16  WristLeft=6
uint16  HandLeft=7
uint16  ShoulderRight=8
uint16  ElbowRight=9
uint16  WristRight=10
uint16  HandRight=11
uint16  HipLeft=12
uint16  KneeLeft=13
uint16  AnkleLeft=14
uint16  FootLeft=15
uint16  HipRight=16
uint16  KneeRight=17
uint16  AnkleRight=18
uint16  FootRight=19
uint16  SpineShoulder=20
uint16  HandTipLeft=21
uint16  ThumbLeft=22
uint16  HandTipRight=23
uint16  ThumbRight=24
uint16  JointCount=25

# Tracking State
uint16  TrackingState_NotTracked=0
uint16  TrackingState_Inferred=1
uint16  TrackingState_Tracked=2

# Hand State
uint16  HandState_Unknown=0
uint16  HandState_NotTracked=1
uint16  HandState_Open=2
uint16  HandState_Closed=3
uint16  HandState_Lasso=3
