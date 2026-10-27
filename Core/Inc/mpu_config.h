#ifndef MPU_CONFIG_H
#define MPU_CONFIG_H

/*
 * Select the connected motion sensor here.
 * Change only MPU_CHIP when replacing the module.
 */
#define MPU_CHIP_MPU6050 6050
#define MPU_CHIP_MPU6500 6500

#ifndef MPU_CHIP
#define MPU_CHIP MPU_CHIP_MPU6050
#endif

#if MPU_CHIP == MPU_CHIP_MPU6050
#define MPU6050
#define MPU_EXPECTED_WHO_AM_I 0x68U
#define MPU_ALTERNATE_WHO_AM_I 0x70U
#define MPU_CHIP_NAME "MPU6050"
#elif MPU_CHIP == MPU_CHIP_MPU6500
#define MPU6500
#define MPU_EXPECTED_WHO_AM_I 0x70U
#define MPU_ALTERNATE_WHO_AM_I 0x00U
#define MPU_CHIP_NAME "MPU6500"
#else
#error "MPU_CHIP must be MPU_CHIP_MPU6050 or MPU_CHIP_MPU6500"
#endif

#define MPU_WHO_AM_I_MATCHES(value) \
	(((value) == MPU_EXPECTED_WHO_AM_I) || \
	 ((value) == MPU_ALTERNATE_WHO_AM_I))

#endif
