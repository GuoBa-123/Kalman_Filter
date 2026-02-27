//
// Created by fish on 2025/10/7.
//

#pragma once

#include <array>
#include <algorithm>

#define BSP_ASSERT(arg) if(!(arg)) ((void)0)//取消断言的实现

namespace math {
    template <int row, int col>
    class matrix {
         template<int r, int c>
    friend class matrix;

        static constexpr float eps = 1e-4;
    public:
        matrix() = default;
        explicit matrix(const float &val) {
            data.fill(0);
            if (val == 0) return;
            for (int i = 0; i < std::min(row, col); i++) {
                data[i * col + i] = val;
            }
        }
        matrix(const matrix &oth) {
            data = oth.data;
        }
        template <typename T>
        matrix(const std::initializer_list <T> &val) {
            BSP_ASSERT(val.size() == row * col);
            auto p = val.begin();
            for (int i = 0; i < row * col; i++) data[i] = *(p++);
        }
        ~matrix() = default;

        static std::pair <int, int> size() {
            return std::make_pair(row, col);
        }

        // 取值
        float &operator() (const int &r, const int &c) {
            BSP_ASSERT(0 <= r and r < row and 0 <= c and c < col);
            return data[r * col + c];
        }
        const float &operator() (const int &r, const int &c) const {
            BSP_ASSERT(0 <= r and r < row and 0 <= c and c < col);
            return data[r * col + c];
        }
        float *operator[] (const int &r) {
            return &data[r * col];
        }
        const float *operator[] (const int &r) const {
            return &data[r * col];
        }

        // 基本运算
        template <typename T>
        matrix operator+ (const T &val) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] + val;
            }
            return ret;
        }
        matrix operator+ (const matrix &oth) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] + oth.data[i];
            }
            return ret;
        }
        matrix operator- () const {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = -data[i];
            }
            return ret;
        }
        matrix operator- (const float &val) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] - val;
            }
            return ret;
        }
        matrix operator- (const matrix &oth) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] - oth.data[i];
            }
            return ret;
        }
        matrix operator* (const float &val) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] * val;
            }
            return ret;
        }
        friend matrix operator* (const float &val, const matrix &oth) {
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = val * oth.data[i];
            }
            return ret;
        }
        template <int el>
        matrix <row, el> operator* (const matrix <col, el> &oth) {
            matrix <row, el> ret(0);
            for (int i = 0; i < row; i++) {
                for (int j = 0; j < el; j++) {
                    for (int k = 0; k < col; k++) {
                        ret[i][j] += (*this)[i][k] * oth[k][j];
                    }
                }
            }
            return ret;
        }
        matrix operator/ (const float &val) {
            if (std::abs(val) < eps) return *this;
            matrix ret;
            for (int i = 0; i < row * col; i++) {
                ret.data[i] = data[i] / val;
            }
            return ret;
        }
        matrix &operator+= (const float &val) {
            for (int i = 0; i < row * col; i++) data[i] += val;
            return *this;
        }
        matrix &operator+= (const matrix &oth) {
            for (int i = 0; i < row * col; i++) {
                data[i] += oth.data[i];
            }
            return *this;
        }
        matrix &operator-= (const float &val) {
            for (int i = 0; i < row * col; i++) {
                data[i] -= val;
            }
            return *this;
        }
        matrix &operator-= (const matrix &oth) {
            for (int i = 0; i < row * col; i++) {
                data[i] -= oth.data[i];
            }
            return *this;
        }
        matrix &operator*= (const float &val) {
            for (int i = 0; i < row * col; i++) {
                data[i] *= val;
            }
            return *this;
        }
        matrix &operator/= (const float &val) {
            if (std::abs(val) > eps)
                for (int i = 0; i < row * col; i++) {
                    data[i] /= val;
                }
            return *this;
        }
        bool operator== (const matrix &oth) {
            for (int i = 0; i < row * col; i++) {
                if (std::abs(data[i] - oth.data[i]) > eps) return false;
            }
            return true;
        }
        bool operator!= (const matrix &oth) {
            return !(*this == oth);
        }
        matrix <col, row> transpose() {
            matrix <col, row> ret;
            for (int i = 0; i < row; i++) {
                for (int j = 0; j < col; j++) {
                    ret.data[j * row + i] = data[i * col + j];
                }
            }
            return ret;
        }
        float trace() {
            float ret = 0;
            for (int i = 0; i < std::min(row, col); i++) {
                ret += data[i * col + i];
            }
            return ret;
        }
        template <int _r = row, int _c = col>
        std::enable_if_t <_r == _c and _r <= 3, matrix> inv() {
            if constexpr (_r == 3) {
                const float a = data[0], b = data[1], c = data[2];
                const float d = data[3], e = data[4], f = data[5];
                const float g = data[6], h = data[7], i = data[8];

                // 余子式（cofactors，未转置）
                const float C00 =  (e * i - f * h);
                const float C01 = -(d * i - f * g);
                const float C02 =  (d * h - e * g);

                const float C10 = -(b * i - c * h);
                const float C11 =  (a * i - c * g);
                const float C12 = -(a * h - b * g);

                const float C20 =  (b * f - c * e);
                const float C21 = -(a * f - c * d);
                const float C22 =  (a * e - b * d);

                // 行列式（按第一行展开）
                float det = a * C00 + b * C01 + c * C02;

                if (std::abs(det) < eps) {
                    // 此时认为矩阵不可逆
                    BSP_ASSERT(false);
                    return matrix(1);
                }
                const float id = 1.0f / det;

                // 逆矩阵 = adj(A) / det = Cofactor^T / det
                matrix ret; // 3x3
                ret.data[0] = C00 * id;  ret.data[1] = C10 * id;  ret.data[2] = C20 * id;
                ret.data[3] = C01 * id;  ret.data[4] = C11 * id;  ret.data[5] = C21 * id;
                ret.data[6] = C02 * id;  ret.data[7] = C12 * id;  ret.data[8] = C22 * id;
                return ret;
            }
            if constexpr (_r == 2) {
                const float a = data[0], b = data[1], c = data[2], d = data[3];
                if (std::abs(a * d - b * c) < eps) {
                    BSP_ASSERT(false);
                    return matrix(1);
                }
                return matrix({d, -b, -c, a}) / (a * d - b * c);
            }
            if (std::abs(data[0]) < eps) {
                BSP_ASSERT(false);
                return matrix({1});
            }
            return matrix({1 / data[0]});
        }
        template <int _r = row, int _c = col>
        std::enable_if_t <_r == _c and (_r > 3), matrix> inv() {
            constexpr int n = _r;

            float A[n*n], I[n*n];
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    A[i*n + j] = data[i * n + j];
                    I[i*n + j] = (i == j) ? 1.0f : 0.0f;
                }
            }

            auto swap_rows = [&](int r1, int r2) {
                if (r1 == r2) return;
                for (int j = 0; j < n; ++j) {
                    std::swap(A[r1*n + j], A[r2*n + j]);
                    std::swap(I[r1*n + j], I[r2*n + j]);
                }
            };

            for (int k = 0; k < n; ++k) {
                // 选主元
                int p = k;
                float maxv = std::abs(A[k*n + k]);
                for (int i = k + 1; i < n; ++i) {
                    float v = std::abs(A[i*n + k]);
                    if (v > maxv) { maxv = v; p = i; }
                }
                swap_rows(k, p);

                float diag = A[k*n + k];

                // 如果主元太小，当作矩阵奇异
                if (std::abs(diag) < eps) {
                    BSP_ASSERT(false);
                    return matrix(1);
                }

                float inv_diag = 1.0f / diag;
                for (int j = 0; j < n; ++j) {
                    A[k*n + j] *= inv_diag;
                    I[k*n + j] *= inv_diag;
                }

                for (int i = 0; i < n; ++i) {
                    if (i == k) continue;
                    float f = A[i*n + k];
                    for (int j = 0; j < n; ++j) {
                        A[i*n + j] -= f * A[k*n + j];
                        I[i*n + j] -= f * I[k*n + j];
                    }
                }
            }

            matrix ret;
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    ret.data[i * n + j] = I[i*n + j];
            return ret;
        }

        static matrix zeros() {
            return matrix(0);
        }
        static matrix ones() {
            matrix ret;
            ret.data.fill(1);
            return ret;
        }
        
    protected:
        std::array <float, row * col> data;
    };
}
