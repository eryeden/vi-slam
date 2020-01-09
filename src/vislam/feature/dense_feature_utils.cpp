#include "dense_feature_extructor.hpp"

using namespace dense_feature;

// 曲率画像を生成する
cv::Mat utils::generate_curvature_image(const cv::Mat &img_gray)
{

/**
 * @brief CV_64FC1とCV_32FC1では32bitのほうが処理が結構早い。
 * あまり結果が変わらないようなら32bitのバージョンで計算するのがよさそう。
 * 
 */
#define USE_CV_FP64
#ifdef USE_CV_FP64
    cv::Mat img_x, img_xx;
    cv::Sobel(img_gray, img_x, CV_64FC1, 1, 0, 5);
    // cv::Sobel(img_x, img_xx, CV_64FC1, 1, 0);
    cv::Sobel(img_gray, img_xx, CV_64FC1, 2, 0, 5);

    cv::Mat img_y, img_yy;
    cv::Sobel(img_gray, img_y, CV_64FC1, 0, 1, 5);
    // cv::Sobel(img_y, img_yy, CV_64FC1, 0, 1);
    cv::Sobel(img_gray, img_yy, CV_64FC1, 0, 2, 5);

    // cv::Mat img_xy;
    // cv::Sobel(img_x, img_xy, CV_64FC1, 0, 1, 5);

    cv::Mat img_xy;
    cv::Sobel(img_gray, img_xy, CV_64FC1, 1, 1, 5);

#else
    cv::Mat img_x, img_xx;
    cv::Sobel(img_gray, img_x, CV_32FC1, 1, 0);
    cv::Sobel(img_x, img_xx, CV_32FC1, 1, 0);

    cv::Mat img_y, img_yy;
    cv::Sobel(img_gray, img_y, CV_32FC1, 0, 1);
    cv::Sobel(img_y, img_yy, CV_32FC1, 0, 1);

    cv::Mat img_xy;
    cv::Sobel(img_x, img_xy, CV_32FC1, 0, 1);
#endif

    cv::Mat img_x_p2;
    cv::multiply(img_x, img_x, img_x_p2);
    cv::Mat img_y_p2;
    cv::multiply(img_y, img_y, img_y_p2);

    cv::Mat term1, term2, term2tmp, term3;
    cv::multiply(img_xx, img_y_p2, term1);

    cv::multiply(img_x, img_y, term2tmp);
    // cv::multiply(term2, img_xy, term2, -2.0);
    cv::multiply(term2tmp, img_xy, term2);

    cv::multiply(img_yy, img_x_p2, term3);

    cv::Mat curv;
    // curv = term1 + term2 * (-2.0) + term3;
    curv = term1 + (-2.0) * term2 + term3;

    return curv;
}

// Local maxの探索を行う
cv::Point2i utils::track_local_max(
    const cv::Mat &img_mono,
    const cv::Point2i &initial_point)
{
    cv::Point2i prev_neigbhor_max = initial_point;

    // std::cout << "########################" << std::endl;
    for (size_t i = 0;; i++)
    {
        // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
        cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
        // std::cout << "Test: " << i << ", " << neighbor_max << std::endl;
        // std::cout << "PrevTest: " << i << ", " << prev_neigbhor_max << std::endl;
        if (neighbor_max == prev_neigbhor_max)
        {
            break;
        }

        prev_neigbhor_max = neighbor_max;
    }

    return prev_neigbhor_max;
}

// 探索開始点をアンカーとしてLocal maxを探索する
cv::Point2i utils::track_local_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2i &initial_point)
{
    cv::Point2i prev_neigbhor_max = initial_point;

    // std::cout << "########################" << std::endl;
    for (size_t i = 0;; i++)
    {
        // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
        cv::Point2i neighbor_max = get_neighbor_max_with_regularization(
            img_mono, prev_neigbhor_max, 0.01, 1, initial_point);
        // std::cout << "Test: " << i << ", " << neighbor_max << std::endl;
        // std::cout << "PrevTest: " << i << ", " << prev_neigbhor_max << std::endl;
        if (neighbor_max == prev_neigbhor_max)
        {
            break;
        }

        prev_neigbhor_max = neighbor_max;
    }
    // cv::Point2i diff = initial_point - prev_neigbhor_max;
    // double norm = cv::norm(diff);
    // printf("Norm: %f\n", norm);

    return prev_neigbhor_max;
}

// 3近傍の最大値探索を行う
cv::Point2i utils::get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point)
{
    // std::cout << "Neigbhor ######### " << std::endl;
    // std::cout << "Neigbhor: " << input_point << std::endl;
    double max = img_mono.at<double>(input_point.y, input_point.x);
    std::vector<int32_t> dxs = {1, -1};
    std::vector<int32_t> dys = {1, -1};
    cv::Point2i max_point = input_point;

    for (const auto dx : dxs)
    {
        for (const auto dy : dys)
        {
            int32_t px, py;
            px = input_point.x + dx;
            py = input_point.y + dy;
            if ((px > 0) && (px < img_mono.size().width) && ((py > 0) && (py < img_mono.size().height)))
            {
                // std::cout << "Neigbhor: " << px << ", " << py << std::endl;
                auto curren_value = img_mono.at<double>(py, px);
                if (max < curren_value)
                {
                    max = curren_value;
                    max_point.x = px;
                    max_point.y = py;
                }
            }
        }
    }

    return max_point;
}

// 3近傍の探索を推定点で正則化し探索する
cv::Point2i utils::get_neighbor_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2i &input_point,
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &estimated_point)
{
    // std::cout << "Neigbhor ######### " << std::endl;
    // std::cout << "Neigbhor: " << input_point << std::endl;
    double max = img_mono.at<double>(input_point.y, input_point.x) +
                 get_regularization_term(lambda_coeff, sigma_coeff, input_point, estimated_point);
    std::vector<int32_t> dxs = {1, -1};
    std::vector<int32_t> dys = {1, -1};
    cv::Point2i max_point = input_point;

    for (const auto dx : dxs)
    {
        for (const auto dy : dys)
        {
            int32_t px, py;
            px = input_point.x + dx;
            py = input_point.y + dy;
            if ((px > 0) && (px < img_mono.size().width) && ((py > 0) && (py < img_mono.size().height)))
            {
                // std::cout << "Neigbhor: " << px << ", " << py << std::endl;
                auto curren_value = img_mono.at<double>(py, px) +
                                    get_regularization_term(lambda_coeff, sigma_coeff, cv::Point2f(px, py), estimated_point);

                if (max < curren_value)
                {
                    max = curren_value;
                    max_point.x = px;
                    max_point.y = py;
                }
            }
        }
    }

    return max_point;
}

//正則化項の計算
double utils::get_regularization_term(
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &input_point,
    const cv::Point2f &estimated_point)
{
    double dest = cv::norm(input_point - estimated_point);
    double rho = dest * dest / (dest * dest + sigma_coeff);
    double omega = 1 - rho;
    double reg = lambda_coeff * omega;

    return reg;
}

// Affine行列をもとに次フレームの特徴点位置を計算する
bool utils::warp_point(
    const cv::Point2f &input,
    const cv::Mat &affine_mat,
    const cv::Size &image_size,
    cv::Point2f &output)
{
    cv::Mat input_mat = (cv::Mat_<double>(3, 1) << input.x, input.y, 1);
    cv::Mat ck_est = affine_mat * input_mat;
    cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

    if ((ck_est_pt.x >= 0) && (ck_est_pt.x < image_size.width) && (ck_est_pt.y >= 0) && (ck_est_pt.y < image_size.height))
    {
        output = ck_est_pt;
        return true;
    }
    else
    {
        output = input;
        return false;
    }
}