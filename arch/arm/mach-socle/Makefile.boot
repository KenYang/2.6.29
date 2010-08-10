#ARM 7
ifeq ($(CONFIG_ARCH_P7DK),y)
   zreladdr-y	:= 0x80008000
params_phys-y	:= 0x80000100
initrd_phys-y	:= 0x80800000
endif

ifeq ($(CONFIG_ARCH_PDK_PC7210),y)
   zreladdr-y   := 0x80008000
params_phys-y   := 0x80000100
initrd_phys-y   := 0x80800000
endif

#ARM 9
ifeq ($(CONFIG_ARCH_LDK3V21),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_CDK),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_MSMV),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_SCDK),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_PDK_PC9002),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_PDK_PC9220),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_PDK_PC9223),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

#Dual
ifeq ($(CONFIG_ARCH_LDK5),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_MDK_3D),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_MDK_FHD),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif
