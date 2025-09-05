#pragma once

#include "AxisAngle.hpp"
#include "Dcm.hpp"
#include "Dcm2.hpp"
#include "Dual.hpp"
#include "Euler.hpp"
#include "helper_functions.hpp"
#include "LeastSquaresSolver.hpp"
#include "Matrix.hpp"
#include "PseudoInverse.hpp"
#include "Quaternion.hpp"
#include "Scalar.hpp"
#include "Slice.hpp"
#include "SparseVector.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"

static constexpr float CONSTANTS_ONE_G = 9.80665f;						        // m/s^2
static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
static constexpr float  CONSTANTS_RADIUS_OF_EARTH_F = CONSTANTS_RADIUS_OF_EARTH;// meters (m)
static constexpr float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;				// radians/second (rad/s)