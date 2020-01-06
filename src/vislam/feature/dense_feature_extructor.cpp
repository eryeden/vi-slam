#include "dense_feature_extructor.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <omp.h>

dense_feature::dense_feature(uint64_t id_, const Eigen::Vector2i &feature_point)
    : id(id_), feature_history({feature_point})
{
    ;
}

dense_feature::dense_feature()
    : id(std::numeric_limits<uint64_t>::max())
{
}

void dense_feature::add_feature(const Eigen::Vector2i &feature)
{
    feature_history.emplace_back(feature);
}
const Eigen::Vector2i dense_feature::get_latest_feature() const
{
    return *(feature_history.end());
}
const std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> &dense_feature::get_feature_history() const
{
    return feature_history;
}

uint64_t dense_feature::get_id() const
{
    return id;
}
void dense_feature::set_id(const uint64_t id_)
{
    id = id_;
}

dense_feature_extructor::dense_feature_extructor()
{
    is_initialize = true;
    is_initialize_dominant_flow = true;
}

void dense_feature_extructor::run_extruction(const std::string &path_to_log_dir)
{
    LogPlayer_extended lpe(path_to_log_dir, 0.01);

    for (size_t i = 0; i < static_cast<uint32_t>(lpe.get_frame_size()); i++)
    {
        /**
             * @brief Log dataから画像を読み込む
             * 
             */
        cv::Mat img;
        double timestamp;
        lpe.get_frame_by_index(img, timestamp, i);
        cv::Mat img_show;
        img.copyTo(img_show);

        // 曲率画像を生成
        cv::Mat outimg = get_curavture(img);

        // dominant flowを計算
        cv::Mat dominant_affine = get_dominant_flow(img);

        // 一様分布で特徴点初期位置
        int32_t num_points = 10000;
        if (is_initialize)
        {
            feature_points.clear();
            feature_points.resize(num_points);
        }

        std::random_device rnd;                                                          // 非決定的な乱数生成器を生成
        std::mt19937 mt(rnd());                                                          //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<int32_t> rand_width(0, outimg.size().width - 1);   // [0, 99] 範囲の一様乱数
        std::uniform_int_distribution<int32_t> rand_height(0, outimg.size().height - 1); // [0, 99] 範囲の一様乱数

#pragma omp parallel for
        for (size_t i = 0; i < num_points; i++)
        {
            if (is_initialize == true)
            {
                feature_points[i].x = rand_width(mt);
                feature_points[i].y = rand_height(mt);
            }
            else
            {
                bool is_inside = warp_point(feature_points[i], dominant_affine, img.size(), feature_points[i]);
            }

            feature_points[i] = track_local_max(outimg, feature_points[i]);
            // std::cout << feature_points[i] << std::endl;
            cv::circle(img_show, feature_points[i], 1, cv::Scalar(255, 0, 0), 1);
        }

        if (i > 10)
        {
            is_initialize = false;
        }

        cv::Mat outimg_normed;
        cv::normalize(outimg, outimg_normed, 0, 1, cv::NORM_MINMAX);
        cv::imshow("Test", outimg_normed);
        cv::imshow("Feature", img_show);
        cv::waitKey(10);
    }
}

void dense_feature_extructor::run_extruction_cam(const std::string &path_to_cam, double scale)
{
    // LogPlayer_extended lpe(path_to_log_dir, 0.01);

    cv::VideoCapture cam(path_to_cam);

    for (size_t i = 0;; i++)
    {
        cv::Mat img, tmp_img;
        double timestamp;
        cam >> tmp_img;
        cv::resize(tmp_img, img, cv::Size(), scale, scale, CV_INTER_LINEAR);
        cv::Mat img_show;
        img.copyTo(img_show);

        // 曲率画像を生成
        cv::Mat outimg = get_curavture(img);

        // dominant flowを計算
        cv::Mat dominant_affine = get_dominant_flow(img);

        // 一様分布で特徴点初期位置
        int32_t num_points = 10000;
        if (is_initialize)
        {
            feature_points.clear();
            feature_points.resize(num_points);
        }

        std::random_device rnd;                                                          // 非決定的な乱数生成器を生成
        std::mt19937 mt(rnd());                                                          //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_int_distribution<int32_t> rand_width(0, outimg.size().width - 1);   // [0, 99] 範囲の一様乱数
        std::uniform_int_distribution<int32_t> rand_height(0, outimg.size().height - 1); // [0, 99] 範囲の一様乱数

        //#pragma omp parallel for
        for (size_t i = 0; i < num_points; i++)
        {
            if (is_initialize == true)
            {
                feature_points[i].x = rand_width(mt);
                feature_points[i].y = rand_height(mt);
            }
            else
            {
                bool is_inside = warp_point(feature_points[i], dominant_affine, img.size(), feature_points[i]);
            }

            feature_points[i] = track_local_max(outimg, feature_points[i]);
            // std::cout << feature_points[i] << std::endl;
            cv::circle(img_show, feature_points[i], 2, cv::Scalar(255, 0, 0), 1, CV_AA);
        }

        if (i > 10)
        {
            is_initialize = false;
        }

        cv::Mat outimg_normed;
        cv::normalize(outimg, outimg_normed, 0, 1, cv::NORM_MINMAX);
        cv::imshow("Test", outimg_normed);
        cv::imshow("Feature", img_show);
        cv::waitKey(1);
    }
}



void dense_feature_extructor::detect_and_track(const cv::Mat &input_colored)
{

}

cv::Mat dense_feature_extructor::get_curavture(const cv::Mat &input_color)
{
    cv::Mat img_gray;
    cv::cvtColor(input_color, img_gray, CV_BGR2GRAY);

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
    // cv::Sobel(img_x, img_xy, CV_64FC1, 0, 1);

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

    cv::Mat term1, term2, term3;
    cv::multiply(img_xx, img_y_p2, term1);

    cv::multiply(img_x, img_y, term2);
    // cv::multiply(term2, img_xy, term2, -2.0);
    cv::multiply(term2, img_xy, term2);

    cv::multiply(img_yy, img_x_p2, term3);

    cv::Mat curv;
    curv = term1 + term2 * (-2.0) + term3;

    return curv;
    // return term3;
    // return img_x;
}

cv::Point2i dense_feature_extructor::get_neighbor_max(
    const cv::Mat &img_mono,
    const cv::Point2i &input_point)
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

cv::Point2i dense_feature_extructor::get_neighbor_max_with_regularization(
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

cv::Point2i dense_feature_extructor::track_local_max(
    const cv::Mat &img_mono,
    const cv::Point2i &initial_point)
{
    cv::Point2i prev_neigbhor_max = initial_point;

    // std::cout << "########################" << std::endl;
    for (size_t i = 0;; i++)
    {
        // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
        cv::Point2i neighbor_max = get_neighbor_max_with_regularization(
            img_mono, prev_neigbhor_max, 0.1, 5, initial_point);
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

cv::Mat dense_feature_extructor::get_dominant_flow(const cv::Mat &img_color)
{

    // モノクロ化とサイズをちっさくする
    cv::Mat input_small, img_draw, img_mono;
    img_color.copyTo(img_draw);
    cv::cvtColor(img_color, img_mono, CV_BGR2GRAY);
    double scale = 1.0 / 3.0;
    cv::resize(img_mono, input_small, cv::Size(), scale, scale, CV_INTER_LINEAR);

    // Feature pointを抽出する
    std::vector<cv::Point2f> points_from_input;
    cv::goodFeaturesToTrack(input_small, points_from_input, 1000, 0.01, 0.1);
    std::vector<cv::KeyPoint> current_keypoints;
    cv::KeyPoint::convert(points_from_input, current_keypoints);

    // 特徴量を計算する
    auto feature = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
    cv::Mat current_descriptor;
    feature->compute(input_small, current_keypoints, current_descriptor);

    // feature matching、最初のフレームは計算しない
    std::vector<cv::DMatch> good_matches;
    cv::Mat est_affine;
    if (!is_initialize_dominant_flow)
    {
        // auto matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(current_descriptor, prev_descriptor, knn_matches, 2);

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.5f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        // Affineの推定を行う
        std::vector<cv::Point2f> keypoints_src, keypoints_dst;
        for (const auto &good_match : good_matches)
        {
            auto ck = current_keypoints[good_match.queryIdx].pt * (1.0 / scale);
            auto pk = prev_keypoints[good_match.trainIdx].pt * (1.0 / scale);
            keypoints_src.emplace_back(pk);
            keypoints_dst.emplace_back(ck);
        }
        est_affine = cv::estimateRigidTransform(keypoints_src, keypoints_dst, false);
    }

    cv::Mat match_img;
    // Feature pointを描画する
    if (is_initialize_dominant_flow)
    {
    }
    else
    {
        for (const auto &p : points_from_input)
        {
            cv::circle(img_draw, p * (1.0 / scale), 1, cv::Scalar(0, 0, 255), 1);
        }
        for (const auto &good_match : good_matches)
        {
            auto ck = current_keypoints[good_match.queryIdx].pt * (1.0 / scale);
            auto pk = prev_keypoints[good_match.trainIdx].pt * (1.0 / scale);
            cv::Mat pk_mat = (cv::Mat_<double>(3, 1) << pk.x, pk.y, 1);

            cv::Mat ck_est = est_affine * pk_mat;
            cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

            // auto ck = current_keypoints[good_match.trainIdx].pt * (1.0 / scale);
            // auto pk = prev_keypoints[good_match.queryIdx].pt * (1.0 / scale);

            // cv::circle(img_draw, ck, 2, cv::Scalar(255, 0, 0), 1, CV_AA);
            // cv::circle(img_draw, pk, 2, cv::Scalar(0, 255, 0), 1, CV_AA);
            // cv::line(img_draw, ck, pk, cv::Scalar(255, 255, 255), 1, CV_AA);
            // cv::line(img_draw, pk, ck_est_pt, cv::Scalar(255, 0, 255), 1, CV_AA);
            // cv::circle(img_draw, ck_est_pt, 2, cv::Scalar(0, 0, 255), 1, CV_AA);
        }

        // cv::drawMatches(input_small, current_keypoints, prev_img, prev_keypoints, good_matches, match_img);
        // cv::imshow("match", match_img);
    }

    // cv::imshow("feature", img_draw);

    // 後処理
    if (is_initialize_dominant_flow)
    {
        is_initialize_dominant_flow = false;
    }
    prev_descriptor = current_descriptor;
    prev_keypoints = current_keypoints;
    input_small.copyTo(prev_img);

    return est_affine;
}

double dense_feature_extructor::get_regularization_term(
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

std::vector<cv::Point2f> dense_feature_extructor::warp_points(
    const std::vector<cv::Point2f> &inputs,
    cv::Mat &affine_mat,
    cv::Size image_size)
{
    std::vector<cv::Point2f> outputs;
    outputs.reserve(inputs.size());

    for (const auto input : inputs)
    {
        cv::Mat input_mat = (cv::Mat_<double>(3, 1) << input.x, input.y, 1);

        cv::Mat ck_est = affine_mat * input_mat;
        cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

        if ((ck_est_pt.x >= 0) && (ck_est_pt.x < image_size.width) && (ck_est_pt.y >= 0) && (ck_est_pt.y < image_size.height))
        {
            outputs.emplace_back(ck_est_pt);
        }
    }

    return outputs;
}

bool dense_feature_extructor::warp_point(
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
