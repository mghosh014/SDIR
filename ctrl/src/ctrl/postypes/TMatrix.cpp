#include "TMatrix.h"
#include <math.h>

//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!

TMatrix::TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen) {
	m_transformation[0][0] = _one;
	m_transformation[0][1] = _two;
	m_transformation[0][2] = _three;
	m_transformation[0][3] = _four;
	m_transformation[1][0] = _five;
	m_transformation[1][1] = _six;
	m_transformation[1][2] = _seven;
	m_transformation[1][3] = _eight;
	m_transformation[2][0] = _nine;
	m_transformation[2][1] = _ten;
	m_transformation[2][2] = _eleven;
	m_transformation[2][3] = _twelve;
	m_transformation[3][0] = _thirteen;
	m_transformation[3][1] = _fourteen;
	m_transformation[3][2] = _fifteen;
	m_transformation[3][3] = _sixteen;
}


TMatrix::TMatrix(double _trans[6]) {
	m_transformation[0][0] = cos(_trans[3]) * cos(_trans[4]);
//TODO implement

}


TMatrix::TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z) {
	m_transformation[0][0] = cos(_rot_x) * cos(_rot_y);
//TODO implement
}