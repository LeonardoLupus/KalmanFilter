#pragma once

#include <matrix/matrix.h>
#include <vector>

class KalmanFilter
{
    unsigned int m_dimension;
    Matrix<double> m_X;

public:
    Matrix<double> F; // Матрица перехода состояния
    Matrix<double> H; // Матрица наблюдения
    Matrix<double> Q; // Ковариационная матрица шума процесса
    Matrix<double> R; // Ковариационная матрица шума измерения
    Matrix<double> P; // Ковариационная матрица ошибки оценки

    KalmanFilter(unsigned int dimension);
    ~KalmanFilter() = default;

    auto init_filter_manual(const std::vector<Matrix<double>>& config) -> bool;
    // auto init_filter_file() -> bool;

    auto update(Matrix<double> new_data) -> void;
    auto update(std::vector<double> new_data) -> void;

    auto get_vector() const -> std::vector<double>;
    auto get_matrix() const -> Matrix<double>;
};