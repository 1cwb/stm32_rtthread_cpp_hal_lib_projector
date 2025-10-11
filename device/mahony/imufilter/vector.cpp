/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/  >.
 */

#include "vector.hpp"
#include "axis.hpp"
#include <algorithm>
#include <cmath>

namespace bfimu {

// Vector2 implementation
bool Vector2::operator==(const Vector2& other) const {
    return std::equal(v.begin(), v.end(), other.v.begin());
}

Vector2& Vector2::zero() {
    std::fill(v.begin(), v.end(), 0.0f);
    return *this;
}

Vector2 Vector2::operator+(const Vector2& other) const {
    return {v[0] + other.v[0], v[1] + other.v[1]};
}

Vector2 Vector2::operator-(const Vector2& other) const {
    return {v[0] - other.v[0], v[1] - other.v[1]};
}

Vector2 Vector2::operator*(float scalar) const {
    return {v[0] * scalar, v[1] * scalar};
}

Vector2 operator*(float scalar, const Vector2& vec) {
    return vec * scalar;
}

Vector2& Vector2::operator+=(const Vector2& other) {
    v[0] += other.v[0];
    v[1] += other.v[1];
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& other) {
    v[0] -= other.v[0];
    v[1] -= other.v[1];
    return *this;
}

Vector2& Vector2::operator*=(float scalar) {
    v[0] *= scalar;
    v[1] *= scalar;
    return *this;
}

float Vector2::dot(const Vector2& other) const {
    return v[0] * other.v[0] + v[1] * other.v[1];
}

float Vector2::cross(const Vector2& other) const {
    return v[0] * other.v[1] - v[1] * other.v[0];
}

float Vector2::normSq() const {
    return dot(*this);
}

float Vector2::norm() const {
    return std::sqrt(normSq());
}

Vector2& Vector2::normalize() {
    const float normVal = norm();
    if (normVal > std::numeric_limits<float>::epsilon()) {
        *this *= (1.0f / normVal);
    } else {
        zero();
    }
    return *this;
}

Vector2 Vector2::normalized() const {
    Vector2 result = *this;
    result.normalize();
    return result;
}

Vector2& Vector2::rotate(float angle) {
    const float cosAngle = cos_approx(angle);
    const float sinAngle = sin_approx(angle);
    const float newX = v[0] * cosAngle - v[1] * sinAngle;
    const float newY = v[0] * sinAngle + v[1] * cosAngle;
    v[0] = newX;
    v[1] = newY;
    return *this;
}

Vector2 Vector2::rotated(float angle) const {
    Vector2 result = *this;
    result.rotate(angle);
    return result;
}

// Vector3 implementation
bool Vector3::operator==(const Vector3& other) const {
    return std::equal(v.begin(), v.end(), other.v.begin());
}

Vector3& Vector3::zero() {
    std::fill(v.begin(), v.end(), 0.0f);
    return *this;
}

Vector3 Vector3::operator+(const Vector3& other) const {
    return {v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]};
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return {v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]};
}

Vector3 Vector3::operator*(float scalar) const {
    return {v[0] * scalar, v[1] * scalar, v[2] * scalar};
}

Vector3 operator*(float scalar, const Vector3& vec) {
    return vec * scalar;
}

Vector3& Vector3::operator+=(const Vector3& other) {
    v[0] += other.v[0];
    v[1] += other.v[1];
    v[2] += other.v[2];
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    v[0] -= other.v[0];
    v[1] -= other.v[1];
    v[2] -= other.v[2];
    return *this;
}

Vector3& Vector3::operator*=(float scalar) {
    v[0] *= scalar;
    v[1] *= scalar;
    v[2] *= scalar;
    return *this;
}

float Vector3::dot(const Vector3& other) const {
    return v[0] * other.v[0] + v[1] * other.v[1] + v[2] * other.v[2];
}

Vector3 Vector3::cross(const Vector3& other) const {
    return {
        v[1] * other.v[2] - v[2] * other.v[1],
        v[2] * other.v[0] - v[0] * other.v[2],
        v[0] * other.v[1] - v[1] * other.v[0]
    };
}

float Vector3::normSq() const {
    return dot(*this);
}

float Vector3::norm() const {
    return std::sqrt(normSq());
}

Vector3& Vector3::normalize() {
    const float normVal = norm();
    if (normVal > std::numeric_limits<float>::epsilon()) {
        *this *= (1.0f / normVal);
    } else {
        zero();
    }
    return *this;
}

Vector3 Vector3::normalized() const {
    Vector3 result = *this;
    result.normalize();
    return result;
}

// Matrix33 implementation
Matrix33::Matrix33() {
    // Initialize as identity matrix using std::array initialization
    m = {{
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    }};
}

Matrix33::Matrix33(const std::array<std::array<float, 3>, 3>& arr) : m(arr) {}

Matrix33 Matrix33::identity() {
    return Matrix33();
}

Vector3 Matrix33::operator*(const Vector3& vec) const {
    return {
        m[0][0] * vec.v[0] + m[0][1] * vec.v[1] + m[0][2] * vec.v[2],
        m[1][0] * vec.v[0] + m[1][1] * vec.v[1] + m[1][2] * vec.v[2],
        m[2][0] * vec.v[0] + m[2][1] * vec.v[1] + m[2][2] * vec.v[2]
    };
}

Matrix33 Matrix33::transpose() const {
    Matrix33 result;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            result.m[i][j] = m[j][i];
        }
    }
    return result;
}

Matrix33 Matrix33::buildRotationMatrix(const fp_angles_t& rpy) {
    Matrix33 result;
    
    const float cosx = cos_approx(rpy.angles.roll);
    const float sinx = sin_approx(rpy.angles.roll);
    const float cosy = cos_approx(rpy.angles.pitch);
    const float siny = sin_approx(rpy.angles.pitch);
    const float cosz = cos_approx(rpy.angles.yaw);
    const float sinz = sin_approx(rpy.angles.yaw);

    result.m[0][X] = cosz * cosy;
    result.m[0][Y] = -cosy * sinz;
    result.m[0][Z] = siny;
    result.m[1][X] = sinz * cosx + sinx * cosz * siny;
    result.m[1][Y] = cosz * cosx - sinx * sinz * siny;
    result.m[1][Z] = -sinx * cosy;
    result.m[2][X] = sinx * sinz - cosz * cosx * siny;
    result.m[2][Y] = sinx * cosz + sinz * cosx * siny;
    result.m[2][Z] = cosy * cosx;

    return result;
}

Matrix33 Matrix33::yawToRotationMatrixZ(float yaw) {
    Matrix33 result;
    
    const float sinYaw = sin_approx(yaw);
    const float cosYaw = cos_approx(yaw);

    result.m[0][0] = cosYaw;
    result.m[0][1] = -sinYaw;
    result.m[0][2] = 0.0f;
    result.m[1][0] = sinYaw;
    result.m[1][1] = cosYaw;
    result.m[1][2] = 0.0f;
    result.m[2][0] = 0.0f;
    result.m[2][1] = 0.0f;
    result.m[2][2] = 1.0f;

    return result;
}

Vector3 Matrix33::transform(const Vector3& vec) const {
    return *this * vec;
}

Vector3 Matrix33::transformTranspose(const Vector3& vec) const {
    return {
        m[0][0] * vec.v[0] + m[1][0] * vec.v[1] + m[2][0] * vec.v[2],
        m[0][1] * vec.v[0] + m[1][1] * vec.v[1] + m[2][1] * vec.v[2],
        m[0][2] * vec.v[0] + m[1][2] * vec.v[1] + m[2][2] * vec.v[2]
    };
}

// Free function
Vector3 applyRotationMatrix(const Vector3& v, const Matrix33& rotationMatrix) {
    return rotationMatrix.transformTranspose(v);
}

} // namespace bfimu