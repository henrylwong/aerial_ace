/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm9ds1_reg.h"
#include "utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define    BOOT_TIME            20 //ms

#define    WAIT_TIME_MAG        60 //ms
#define    WAIT_TIME_XL        200 //ms
#define    WAIT_TIME_GY        800 //ms

#define    SAMPLES               5 //number of samples

/* Self test results. */
#define    ST_PASS     0x1U
#define    ST_FAIL     0x0U

/* Self test limits in mgauss @ 12G*/
static const float min_st_mag_limit[] = {1000.0f, 1000.0f,  100.0f};
static const float max_st_mag_limit[] = {3000.0f, 3000.0f, 1000.0f};

/* Self test limits in mg @ 2g*/
static const float min_st_xl_limit[] = {70.0f, 70.0f,  70.0f};
static const float max_st_xl_limit[] = {1500.0f, 1500.0f, 1500.0f};

/* Self test limits in mdps @ 2000 dps*/
static const float min_st_gy_limit[] = {200000.0f, 200000.0f, 200000.0f};
static const float max_st_gy_limit[] = {800000.0f, 800000.0f, 800000.0f};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static sensbus_t mag_bus = {&SENSOR_BUS, LSM9DS1_MAG_I2C_ADD_H, 0, 0};
static sensbus_t imu_bus = {&SENSOR_BUS, LSM9DS1_IMU_I2C_ADD_H, 0, 0};

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
extern sensors_event_t gyro;
extern sensors_event_t accel;
extern sensors_event_t mag;
// static float acceleration_mg[3];
// static float angular_rate_mdps[3];
// static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;

extern stmdev_ctx_t dev_ctx_imu;
extern stmdev_ctx_t dev_ctx_mag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

static void IMU_Read(void);
static int IMU_Setup(void);
static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
// static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void IMU_Read(void) {
	lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

	if (reg.status_imu.xlda && reg.status_imu.gda) {
		/* Read accelerometer and gyroscope data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration);
		lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate);
		 accel.acceleration.x = (lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[0]) * 9.807) / 1000;
		 accel.acceleration.y = (lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[1]) * 9.807) / 1000;
		 accel.acceleration.z = (lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[2]) * 9.807) / 1000;
		 gyro.gyro.x = (lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) * 2 * M_PI)/360000;
		 gyro.gyro.y = (lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) * 2 * M_PI)/360000;
		 gyro.gyro.z = (lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) * 2 * M_PI)/360000;
//		accel.acceleration.x = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[0]);
//		accel.acceleration.y = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[1]);
//		accel.acceleration.z = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[2]);
//		gyro.gyro.x = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
//		gyro.gyro.y = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
//		gyro.gyro.z = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
	}

	if (reg.status_mag.zyxda) {
		/* Read magnetometer data */
		memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
		lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
		mag.magnetic.x = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[0]) / 10;
		mag.magnetic.y = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[1]) / 10;
		mag.magnetic.z = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[2]) / 10;
	}
}

static int IMU_Setup(void) {
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;

  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

//  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
  if (whoamI.imu != LSM9DS1_IMU_ID) {
	  while (1) {
		  /* Device not found management */
		  return SETUP_FAIL;
	  }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
	  lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  return SETUP_SUCCESS;
}

static int32_t platform_write_imu(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_write_mag(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;
	reg |= 0x80;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	  return 0;
}

static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	  sensbus_t *sensbus = (sensbus_t *)handle;
	  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	  return 0;
}

static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;
	  reg |= 0x80;
	  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	  return 0;
}

static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}


/* USER CODE END 0 */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
