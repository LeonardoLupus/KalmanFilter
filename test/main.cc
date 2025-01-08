#include <kalmanFilter/kalmanFilter.h>
#include <iostream>

int main() {
    KalmanFilter qwe(2);
    Matrix<double> F ({{1, 0},
                       {0, 1}});
    Matrix<double> H ({{1, 0},
                       {0, 1}});
    Matrix<double> Q ({{1, 0},
                       {0, 1}});
    Matrix<double> R ({{1, 0},
                       {0, 1}});
    Matrix<double> P ({{1, 0},
                       {0, 1}});
    std::vector config {F, H, Q, R, P};

    if (qwe.init_filter_manual(config)) {
        std::cout << "Filter is ready!\n";
    }

    qwe.update(std::vector<double>{
        1.0,
        1.0
    });

    std::cout << "Hello, World!\n" << qwe.get_matrix() << "\n";
    return 0;
}
