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

#pragma once

#include <array>
#include <cmath>
#include "maths.hpp"

namespace bfimu {

class Vector2 {
public:
    std::array<float, 2> v;

    // Constructors
    Vector2() : v{0.0f, 0.0f} {}
    Vector2(float x, float y) : v{x, y} {}
    explicit Vector2(const std::array<float, 2>& arr) : v(arr) {}

    // Accessors
    float& x() { return v[0]; }
    float& y() { return v[1]; }
    const float& x() const { return v[0]; }
    const float& y() const { return v[1]; }
    float& operator[](size_t idx) { return v[idx]; }
    const float& operator[](size_t idx) const { return v[idx]; }

    // Operations
    bool operator==(const Vector2& other) const;
    bool operator!=(const Vector2& other) const { return !(*this == other); }

    Vector2& zero();
    Vector2 operator+(const Vector2& other) const;
    Vector2 operator-(const Vector2& other) const;
    Vector2 operator*(float scalar) const;
    Vector2& operator+=(const Vector2& other);
    Vector2& operator-=(const Vector2& other);
    Vector2& operator*=(float scalar);

    float dot(const Vector2& other) const;
    float cross(const Vector2& other) const;
    float normSq() const;
    float norm() const;
    Vector2& normalize();
    Vector2 normalized() const;
    Vector2& rotate(float angle);  // angle in radians, positive = counterclockwise
    Vector2 rotated(float angle) const;
};

class Vector3 {
public:
    std::array<float, 3> v;

    // Constructors
    Vector3() : v{0.0f, 0.0f, 0.0f} {}
    Vector3(float x, float y, float z) : v{x, y, z} {}
    explicit Vector3(const std::array<float, 3>& arr) : v(arr) {}
    Vector3& operator=(const Vector3& other) {
        if (this != &other) {
            v = other.v;
        }
        return *this;
    }
    Vector3& operator=(std::array<float, 3>&& arr) {
        v = std::move(arr);
        return *this;
    }
    // Accessors
    float& x() { return v[0]; }
    float& y() { return v[1]; }
    float& z() { return v[2]; }
    const float& x() const { return v[0]; }
    const float& y() const { return v[1]; }
    const float& z() const { return v[2]; }
    float& operator[](size_t idx) { return v[idx]; }
    const float& operator[](size_t idx) const { return v[idx]; }

    // Operations
    bool operator==(const Vector3& other) const;
    bool operator!=(const Vector3& other) const { return !(*this == other); }

    Vector3& zero();
    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(float scalar) const;
    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(float scalar);

    float dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
    float normSq() const;
    float norm() const;
    Vector3& normalize();
    Vector3 normalized() const;
};

class Matrix33 {
public:
    std::array<std::array<float, 3>, 3> m;

    // Constructors
    Matrix33();
    explicit Matrix33(const std::array<std::array<float, 3>, 3>& arr);
    static Matrix33 identity();

    // Accessors
    std::array<float, 3>& operator[](size_t idx) { return m[idx]; }
    const std::array<float, 3>& operator[](size_t idx) const { return m[idx]; }

    // Operations
    Vector3 operator*(const Vector3& v) const;
    Matrix33 transpose() const;
    
    // Factory methods
    static Matrix33 buildRotationMatrix(Matrix33& result, const fp_angles_t& rpy);
    static Matrix33 yawToRotationMatrixZ(float yaw);
    
    // Transformation
    Vector3 transform(const Vector3& v) const;
    Vector3 transformTranspose(const Vector3& v) const;
};

// Free functions for compatibility with existing code
Vector3 applyRotationMatrix(const Vector3& v, const Matrix33& rotationMatrix);

} // namespace bfimu