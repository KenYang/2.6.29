//register address define
#define SOCLE_VIP_AHBR_CTRL         0x0000
#define SOCLE_VIP_INT_MASK          0x0004
#define SOCLE_VIP_INT_STS           0x0008
#define SOCLE_VIP_STS               0x000C
#define SOCLE_VIP_CTRL              0x0010
#define SOCLE_VIP_CAPTURE_F1SA_Y    0x0014
#define SOCLE_VIP_CAPTURE_F1SA_Cb   0x0018
#define SOCLE_VIP_CAPTURE_F1SA_Cr   0x001C
#define SOCLE_VIP_CAPTURE_F2SA_Y    0x0020
#define SOCLE_VIP_CAPTURE_F2SA_Cb   0x0024
#define SOCLE_VIP_CAPTURE_F2SA_Cr   0x0028
#define SOCLE_VIP_FB_SR             0x002C
#define SOCLE_VIP_FS                0x0030
#define SOCLE_VIP_CROP              0x0038
#define SOCLE_VIP_CRM               0x003C
#define SOCLE_VIP_RESET             0x0040
#define SOCLE_VIP_L_SFT             0x0044
//VIP_AHBR_CTRL
#define VIP_AHBR_CTRL_INCR      0x1
#define VIP_AHBR_CTRL_INCR8     0x5
#define VIP_AHBR_CTRL_INCR16    0x7
//VIP_INT_MASK
#define VIP_INT_MASK_CAPTURE_DATA_LINE_END    (0x1<<2)
#define VIP_INT_MASK_CAPTURE_FRAME_LOSS       (0x1<<1)
#define VIP_INT_MASK_CAPTURE_COMPLETE         (0x1<<0)
#define VIP_INT_MASK_DISABLE 0x0
//VIP_INT_STS
#define VIP_INT_STS_CAPTURE_DATA_LINE_END     (0x1<<2)
#define VIP_INT_STS_CAPTURE_FRAME_LOSS        (0x1<<1)
#define VIP_INT_STS_CAPTURE_COMPLETE          (0x1<<0)
//VIP_STS
#define VIP_STS_FIFO_OVERFLOW  (0x1<<0)
//VIP_CTRL
#define VIP_CTRL_FORMAT_PAL           (0x1<<10)
#define VIP_CTRL_FORMAT_NTSC          (0x0<<10)
#define VIP_CTRL_NEGATIVE_EDGE        (0x1<<9)
#define VIP_CTRL_POSITIVE_EDGE        (0x0<<9)
#define VIP_CTRL_PING_PONG_MODE       (0x1<<8)
#define VIP_CTRL_CONTINUOUS_MODE      (0x0<<8)
#define VIP_CTRL_FIELD_1_START        (0x1<<7)
#define VIP_CTRL_FIELD_0_START        (0x0<<7)
#define VIP_CTRL_422_OUTPUT           (0x1<<6)
#define VIP_CTRL_420_OUTPUT           (0x0<<6)
#define VIP_CTRL_ONE_FRAME_STOP       (0x1<<5)
#define VIP_CTRL_RESET                (0x1<<1)   //LDK
#define VIP_CTRL_CAPTURE_EN           (0x1<<0)
#define VIP_CTRL_CAPTURE_DIS          (0x0<<0)
//VIP_FB_SR
#define VIP_FB_SR_FRAME_NUM_SHIFT       8
#define VIP_FB_SR_LATEST_USED_FRAME_2  (0x1<<3)
#define VIP_FB_SR_LATEST_USED_FRAME_1  (0x0<<3)
#define VIP_FB_SR_FRAME_LOSS           (0x1<<2)
#define VIP_FBS_FRAME2_DATA_READY      (0x1<<1)
#define VIP_FBS_FRAME1_DATA_READY      (0x1<<0)
//VIP_FS
#define VIP_FS_WIDTH_SHIFT 16
#define VIP_FS_HEIGHT_SHIFT 0
//VIP_RESET
#define VIP_RESET  0x76543210
