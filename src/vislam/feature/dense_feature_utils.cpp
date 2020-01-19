#include "dense_feature_extructor.hpp"

using namespace dense_feature;

#define USE_CV_FP64
// #define SHOW_CURVATURE

// 曲率画像を生成する
cv::Mat utils::generate_curvature_image(const cv::Mat &img_gray)
{

    // cv::GaussianBlur(img_gray, img_gray, cv::Size(3, 3), 10);

/**
 * @brief CV_64FC1とCV_32FC1では32bitのほうが処理が結構早い。
 * あまり結果が変わらないようなら32bitのバージョンで計算するのがよさそう。
 * 
 */
#ifdef USE_CV_FP64
    // const int32_t size_kernel = 11;
    const int32_t size_kernel = 31;
    cv::Mat img_x, img_xx;
    cv::Sobel(img_gray, img_x, CV_64FC1, 1, 0, size_kernel);
    // cv::Sobel(img_x, img_xx, CV_64FC1, 1, 0);
    cv::Sobel(img_gray, img_xx, CV_64FC1, 2, 0, size_kernel);

    cv::Mat img_y, img_yy;
    cv::Sobel(img_gray, img_y, CV_64FC1, 0, 1, size_kernel);
    // cv::Sobel(img_y, img_yy, CV_64FC1, 0, 1);
    cv::Sobel(img_gray, img_yy, CV_64FC1, 0, 2, size_kernel);

    // cv::Mat img_xy;
    // cv::Sobel(img_x, img_xy, CV_64FC1, 0, 1, 5);

    cv::Mat img_xy;
    cv::Sobel(img_gray, img_xy, CV_64FC1, 1, 1, size_kernel);

#else
    // const int32_t size_kernel = 11;
    const int32_t size_kernel = 3;
    cv::Mat img_x, img_xx;
    cv::Sobel(img_gray, img_x, CV_32FC1, 1, 0, size_kernel);
    cv::Sobel(img_gray, img_xx, CV_32FC1, 2, 0, size_kernel);

    cv::Mat img_y, img_yy;
    cv::Sobel(img_gray, img_y, CV_32FC1, 0, 1, size_kernel);
    cv::Sobel(img_gray, img_yy, CV_32FC1, 0, 2, size_kernel);

    cv::Mat img_xy;
    cv::Sobel(img_gray, img_xy, CV_32FC1, 1, 1, size_kernel);
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

    // cv::Mat mask;
    // non_maxima_suppression(curv, mask, true);
    // cv::imshow("mask", mask);

    // cv::GaussianBlur(curv, curv, cv::Size(31, 31), 1);

    // cv::threshold(curv, curv, 100, 0, CV_THRESH_TOZERO);

#ifdef SHOW_CURVATURE
    cv::Mat outimg_normed;
    cv::normalize(curv, outimg_normed, 0, 1, cv::NORM_MINMAX);
    cv::imshow("Curvature", outimg_normed);
#endif

    return curv;
    // return curv_blur;
}

// Local maxの探索を行う
cv::Point2i utils::track_local_max(
    const cv::Mat &img_mono,
    const cv::Point2f &initial_point)
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
    const cv::Point2f &initial_point)
{
    cv::Point2i prev_neigbhor_max = initial_point;

    // std::cout << "########################" << std::endl;
    for (size_t i = 0;; i++)
    {
        // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
        // cv::Point2i neighbor_max = get_neighbor_max_with_regularization(img_mono, prev_neigbhor_max, 0, 1, initial_point);
        cv::Point2i neighbor_max = get_neighbor_max_with_regularization(img_mono, prev_neigbhor_max, 0.1, 1, initial_point);
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

// void localMaxima(cv::Mat src, cv::Mat &dst, int squareSize)
// {
//     if (squareSize == 0)
//     {
//         dst = src.clone();
//         return;
//     }

//     cv::Mat m0;
//     dst = src.clone();
//     cv::Point maxLoc(0, 0);

//     //1.Be sure to have at least 3x3 for at least looking at 1 pixel close neighbours
//     //  Also the window must be <odd>x<odd>
//     int sqrCenter = (squareSize - 1) / 2;

//     //2.Create the localWindow mask to get things done faster
//     //  When we find a local maxima we will multiply the subwindow with this MASK
//     //  So that we will not search for those 0 values again and again
//     cv::Mat localWindowMask = cv::Mat::zeros(cv::Size(squareSize, squareSize), CV_8U); //boolean
//     localWindowMask.at<unsigned char>(sqrCenter, sqrCenter) = 1;

//     //3.Find the threshold value to threshold the image
//     //this function here returns the peak of histogram of picture
//     //the picture is a thresholded picture it will have a lot of zero values in it
//     //so that the second boolean variable says :
//     //  (boolean) ? "return peak even if it is at 0" : "return peak discarding 0"
//     int thrshld = maxUsedValInHistogramData(dst, false);
//     cv::threshold(dst, m0, thrshld, 1, CV_THRESH_BINARY);

//     //4.Now delete all thresholded values from picture
//     dst = dst.mul(m0);

//     //put the src in the middle of the big array
//     for (int row = sqrCenter; row < dst.size().height - sqrCenter; row++)
//         for (int col = sqrCenter; col < dst.size().width - sqrCenter; col++)
//         {
//             //1.if the value is zero it can not be a local maxima
//             if (dst.at<unsigned char>(row, col) == 0)
//                 continue;
//             //2.the value at (row,col) is not 0 so it can be a local maxima point
//             m0 = dst.colRange(col - sqrCenter, col + sqrCenter + 1).rowRange(row - sqrCenter, row + sqrCenter + 1);
//             minMaxLoc(m0, NULL, NULL, NULL, &maxLoc);
//             //if the maximum location of this subWindow is at center
//             //it means we found the local maxima
//             //so we should delete the surrounding values which lies in the subWindow area
//             //hence we will not try to find if a point is at localMaxima when already found a neighbour was
//             if ((maxLoc.x == sqrCenter) && (maxLoc.y == sqrCenter))
//             {
//                 m0 = m0.mul(localWindowMask);
//                 //we can skip the values that we already made 0 by the above function
//                 col += sqrCenter;
//             }
//         }
// }

void utils::non_maxima_suppression(const cv::Mat &image, cv::Mat &mask, bool remove_plateaus)
{
    // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
    cv::dilate(image, mask, cv::Mat());
    cv::compare(image, mask, mask, cv::CMP_GE);

    // optionally filter out pixels that are equal to the local minimum ('plateaus')
    if (remove_plateaus)
    {
        cv::Mat non_plateau_mask;
        cv::erode(image, non_plateau_mask, cv::Mat());
        cv::compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
        cv::bitwise_and(mask, non_plateau_mask, mask);
    }
}

// 曲率画像を見やすいようにする
cv::Mat utils::visualize_curvature_image(const cv::Mat &input_curvature_image)
{
}
