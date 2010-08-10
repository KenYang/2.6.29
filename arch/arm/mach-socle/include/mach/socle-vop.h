//ioctl command
#define VOP_CTRL 0x4630
#define VOP_SIZE 0x4631

//ioctl argument of VOP_CTRL command
#define VOP_START 1
#define VOP_STOP 0
#define VOP_RESET 0xf

//ioctl argument of VOP_SIZE command
#define FRAMESIZE(w,h) ((w<<10) + h) 
//#define FRAMESIZE_QCIF 0   //176x144
//#define FRAMESIZE_CIF  1   //352x288
//#define FRAMESIZE_QVGA 2   //320x240
//#define FRAMESIZE_VGA  3   //640x480
//#define FRAMESIZE_D1   4   //720x480 or 720*576

//arguments for driver's fumction only
#define FORMAT_NTSC 0
#define FORMAT_PAL 1

#define FRAME1 1
#define FRAME2 2

#define ONE_FRAME 1
#define TWO_FRAME 2

