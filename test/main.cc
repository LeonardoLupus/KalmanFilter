#include <kalmanFilter/kalmanFilter.h>
#include <iostream>

int main() {
    KalmanFilter qwe(2);
    Matrix<double> A ({{1, 0},
                       {0, 1}});
    Matrix<double> H ({{1, 0},
                       {0, 1}});
    Matrix<double> Q ({{1, 0},
                       {0, 1}});
    Matrix<double> R ({{1, 0},
                       {0, 1}});
    Matrix<double> P ({{1, 0},
                       {0, 1}});
    std::vector config {A, H, Q, R, P};

    if (qwe.init_filter_manual(config)) {
        std::cout << "Filter is ready!\n";
    }

    qwe.update({{1, 1}, 1, 2});

    std::cout << "Hello, World!\n";
    return 0;
}
