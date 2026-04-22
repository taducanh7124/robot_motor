#ifndef QMC5883L_Compass
#define QMC5883L_Compass

#include <stdint.h>
#include "main.h"
#include "i2c_software.hpp"

class QMC5883LCompass
{

public:
	QMC5883LCompass();
	void init(GPIO_TypeDef *scl_port, uint16_t scl_pin, GPIO_TypeDef *sda_port, uint16_t sda_pin);
	void setADDR(uint8_t b);
	void setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);
	void setMagneticDeclination(int degrees, uint8_t minutes);
	void setSmoothing(uint8_t steps, bool adv);
	void calibrate();
	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
	void setCalibrationOffsets(float x_offset, float y_offset, float z_offset);
	void setCalibrationScales(float x_scale, float y_scale, float z_scale);
	float getCalibrationOffset(uint8_t index);
	float getCalibrationScale(uint8_t index);
	void clearCalibration();
	void setReset();
	void read();
	int getX();
	int getY();
	int getZ();
	int getAzimuth();
	uint8_t getBearing(int azimuth);
	void getDirection(char *myArray, int azimuth);

private:
	void _writeReg(uint8_t reg, uint8_t val);
	int _get(int index);
	float _magneticDeclinationDegrees = 0;
	bool _smoothUse = false;
	uint8_t _smoothSteps = 5;
	bool _smoothAdvanced = false;
	uint8_t _ADDR = 0x0D;
	int _vRaw[3] = {0, 0, 0};
	int _vHistory[10][3];
	int _vScan = 0;
	long _vTotals[3] = {0, 0, 0};
	int _vSmooth[3] = {0, 0, 0};
	void _smoothing();
	float _offset[3] = {0., 0., 0.};
	float _scale[3] = {1., 1., 1.};
	int _vCalibrated[3];
	void _applyCalibration();
	const char _bearings[16][3] = {
		{' ', ' ', 'N'},
		{'N', 'N', 'E'},
		{' ', 'N', 'E'},
		{'E', 'N', 'E'},
		{' ', ' ', 'E'},
		{'E', 'S', 'E'},
		{' ', 'S', 'E'},
		{'S', 'S', 'E'},
		{' ', ' ', 'S'},
		{'S', 'S', 'W'},
		{' ', 'S', 'W'},
		{'W', 'S', 'W'},
		{' ', ' ', 'W'},
		{'W', 'N', 'W'},
		{' ', 'N', 'W'},
		{'N', 'N', 'W'},
	};

	I2C_software I2C;
};

#endif
