#include "dense_feature_extructor.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

dense_feature_extructor::dense_feature_extructor()
{
    is_initialize = true;
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
    cv::Sobel(img_gray, img_x, CV_64FC1, 1, 0);
    cv::Sobel(img_x, img_xx, CV_64FC1, 1, 0);

    cv::Mat img_y, img_yy;
    cv::Sobel(img_gray, img_y, CV_64FC1, 0, 1);
    cv::Sobel(img_y, img_yy, CV_64FC1, 0, 1);

    // cv::Mat img_xy;
    // cv::Sobel(img_x, img_xy, CV_64FC1, 0, 1);

    cv::Mat img_xy;
    cv::Sobel(img_gray, img_xy, CV_64FC1, 1, 1);
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
    cv::multiply(term2, img_xy, term2, -2.0);

    cv::multiply(img_yy, img_x_p2, term3);

    cv::Mat curv;
    curv = term1 + term2 + term3;

    return curv;
    // return term3;
    // return img_x;
}

cv::Point2i dense_feature_extructor::get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point)
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

cv::Point2i dense_feature_extructor::track_local_max(const cv::Mat &img_mono, const cv::Point2i &initial_point)
{
    cv::Point2i prev_neigbhor_max = initial_point;

    // std::cout << "########################" << std::endl;
    for (size_t i = 0;; i++)
    {
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

void dense_feature_extructor::get_dominant_flow(const cv::Mat &img_mono)
{
}