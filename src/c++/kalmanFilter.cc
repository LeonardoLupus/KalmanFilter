#include <kalmanFilter/kalmanFilter.h> 

#include <stdexcept>

KalmanFilter::KalmanFilter(unsigned int dimension): m_dimension(dimension)
{
    F.resize(m_dimension, m_dimension);
    H.resize(m_dimension, m_dimension);
    Q.resize(m_dimension, m_dimension);
    R.resize(m_dimension, m_dimension);
    P.resize(m_dimension, m_dimension);
    m_X.resize(1, m_dimension);
}

auto KalmanFilter::init_filter_manual(const std::vector<Matrix<double>>& config) -> bool
{
    if (config.size() != 5)
    {
        throw std::runtime_error("Error! Number of manual entered Matrix is not 5!");
    }
    if (config[0].number_col() != m_dimension || config[0].number_row() != m_dimension)
    {
        throw std::runtime_error("Error! Dimension of manual entered Matrix F Wrong!");
    }
    if (config[1].number_col() != m_dimension || config[1].number_row() != m_dimension)
    {
        throw std::runtime_error("Error! Dimension of manual entered Matrix H Wrong!");
    }
    if (config[2].number_col() != m_dimension || config[2].number_row() != m_dimension)
    {
        throw std::runtime_error("Error! Dimension of manual entered Matrix Q Wrong!");
    }
    if (config[3].number_col() != m_dimension || config[3].number_row() != m_dimension)
    {
        throw std::runtime_error("Error! Dimension of manual entered Matrix R Wrong!");
    }
    if (config[4].number_col() != m_dimension || config[4].number_row() != m_dimension)
    {
        throw std::runtime_error("Error! Dimension of manual entered Matrix P Wrong!");
    }
    
    F = config[0];
    H = config[1];
    Q = config[2];
    R = config[3];
    P = config[4];

    return true;
}

auto KalmanFilter::update(Matrix<double> new_data) -> void
{
    // Прогноз
    m_X = F * m_X;
    P = F * P * F.Tr() + Q;

    // Обновление
    auto K = P * H.Tr() * (H * P * H.Tr() + R).inverse();
    m_X = m_X + K * (new_data - H * m_X);
    P = (matrix::identity_matrix<double>(m_dimension) - K * H) * P;
}

auto KalmanFilter::update(std::vector<double> new_data) -> void
{
    Matrix buffer {new_data, 1, new_data.size()};
    update(buffer);
}

auto KalmanFilter::get_vector() const -> std::vector<double> 
{
    std::vector<double> buffer;
    for (size_t i = 0; i < m_X.number_row(); i++)
    {
        for (size_t j = 0; j < m_X.number_col(); j++)
        {
            buffer.emplace_back(m_X(j, i));
        }
    }
    
    return buffer;
}

auto KalmanFilter::get_matrix() const -> Matrix<double>
{
    return m_X;
}