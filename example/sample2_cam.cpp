#include <iostream>
#include <opencv2/opencv.hpp>
// #include "dense_feature_extructor_proto.hpp"
#include "dense_feature_extructor.hpp"

#include "log_util.h"

cv::Scalar HSVtoRGB(double H, double S, double V)
{
    double C = S * V;
    double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
    double m = V - C;
    double Rs, Gs, Bs;

    if (H >= 0 && H < 60)
    {
        Rs = C;
        Gs = X;
        Bs = 0;
    }
    else if (H >= 60 && H < 120)
    {
        Rs = X;
        Gs = C;
        Bs = 0;
    }
    else if (H >= 120 && H < 180)
    {
        Rs = 0;
        Gs = C;
        Bs = X;
    }
    else if (H >= 180 && H < 240)
    {
        Rs = 0;
        Gs = X;
        Bs = C;
    }
    else if (H >= 240 && H < 300)
    {
        Rs = X;
        Gs = 0;
        Bs = C;
    }
    else
    {
        Rs = C;
        Gs = 0;
        Bs = X;
    }

    return cv::Scalar((Rs + m) * 255, (Gs + m) * 255, (Bs + m) * 255);
}

int main()
{

    /**
     * @brief 描画用の色を生成する
     * 
     */
    uint32_t num_colors = 1000;
    std::vector<cv::Scalar> colors(num_colors);
    for (size_t i = 0; i < colors.size(); i++)
    {
        double h, s, v;
        h = static_cast<double>(i) / static_cast<double>(num_colors) * 360.0;
        colors[i] = HSVtoRGB(h, 1, 1);
    }

    dense_feature::dense_feature_extructor dfe(0.1, 0.1);

    // cv::VideoCapture cap("/dev/video0");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_4257.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_4134.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_4240.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_4287.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_4306.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_5144.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_5151.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_5156.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/IMG_5162.MOV");
    // cv::VideoCapture cap("/home/ery/Devel/tmp/assets/test_fukuroi_ryou303.mp4");
    cv::VideoCapture cap("/home/ery/Devel/tmp/assets/imgs_jetson/frame%d.jpg");

    int64_t ref_size = 5;
    // double scale = 1.0 / 2;
    double scale = 1.0;

    cv::Mat tmp, tmp_resized;
    cap >> tmp;
    cv::resize(tmp, tmp_resized, cv::Size(), scale, scale);

// #define REC
#ifdef REC
    cv::VideoWriter wrt("test.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, tmp_resized.size());
    for (size_t i = 0; i < 1000; i++)
#else
    for (size_t i = 0;; i++)
#endif

    {
        cv::Mat img, img_tmp;
        cap >> img_tmp;
        cv::resize(img_tmp, img, cv::Size(), scale, scale);

        dfe.detect_and_track(img);

        // for (size_t fnum = std::max(static_cast<int64_t>(dfe.features.size()) - ref_size, 0l);
        //      fnum < dfe.features.size(); fnum++)
        // {
        //     for (size_t i = 0; i < dfe.features[fnum].features.size(); i++)
        //     {
        //         auto &f = dfe.features[fnum];
        //         cv::circle(img, cv::Point2i(f.features[i][0], f.features[i][1]), 1, colors[f.featureIDs[i] % num_colors], 1);
        //     }
        // }

        std::map<uint64_t, std::vector<cv::Point2i>> feature_lists;
        for (size_t i = 0; i < (dfe.features[dfe.features.size() - 1]).features.size(); i++)
        {
            auto &f = dfe.features[dfe.features.size() - 1];
            feature_lists[f.featureIDs[i]] = std::vector<cv::Point2i>({cv::Point2i(f.features[i][0], f.features[i][1])});
        }

        for (size_t fnum = dfe.features.size() - 1; fnum > std::max(static_cast<int64_t>(dfe.features.size()) - ref_size, 0l); fnum--)
        {
            for (size_t i = 0; i < dfe.features[fnum].features.size(); i++)
            {
                auto &f = dfe.features[fnum];
                if (feature_lists.count(f.featureIDs[i]) != 0)
                {
                    feature_lists[f.featureIDs[i]].emplace_back(cv::Point2i(f.features[i][0], f.features[i][1]));
                }
            }
        }

        double maxlen = 0;
        std::vector<double> lens(0);
        for (const auto &[id, p] : feature_lists)
        {
            cv::Point2i d = p[0] - p[p.size() - 1];
            double dist = cv::norm(d) / p.size();
            // cv::Point2i d = p[0] - p[1];

            lens.emplace_back(cv::norm(dist));
            // if (maxlen < cv::norm(d))
            // {
            //     maxlen = cv::norm(d);
            // }
        }
        double len_sum = std::accumulate(std::begin(lens), std::end(lens), 0.0);
        double len_ave = len_sum / lens.size();
        double len_var = std::inner_product(std::begin(lens), std::end(lens), std::begin(lens), 0.0) / lens.size() - len_ave * len_ave;
        for (const auto l : lens)
        {
            if ((maxlen < l) && (l < (1.0 * std::sqrt(len_var)) + len_ave))
            {
                maxlen = l;
            }
        }

        img *= 0.7;

        for (const auto &[id, p] : feature_lists)
        {
            cv::Point2i d = p[0] - p[p.size() - 1];
            // cv::Point2i d = p[0] - p[1];
            double angle = std::atan2(d.y, d.x) * 180.0 / M_PI;
            // double len = cv::norm(d);
            double len = cv::norm(d) / p.size();
            angle += 180;

            // cv::Scalar dcolor = HSVtoRGB(angle, 1, 1);
            cv::Scalar dcolor = HSVtoRGB(len / maxlen * 360.0, 1, 1);
            // cv::Scalar dcolor = colors[id % num_colors];

            // cv::polylines(img_color, p, false, colors[id % num_colors]);
            if (len < (2.0 * std::sqrt(len_var) + len_ave))
            {
                // cv::polylines(img, p, false, dcolor, 1);
                cv::circle(img, p[0], 2, dcolor, 1);
            }
        }

        cv::imshow("feature", img);
        cv::waitKey(1);
#ifdef REC
        wrt << img;
#endif
    }
}