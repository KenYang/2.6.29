#include <media/soc_camera.h>

struct socle_vip_camera_info {
	unsigned long flags; /* SOCAM_... */
	void (*enable_camera)(void);
	void (*disable_camera)(void);
};

