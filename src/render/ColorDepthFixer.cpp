#include "ColorDepthFixer.h"
#include "utils/MathUtil.h"
#include "utils/DefUtil.h"
#include "utils/LogUtil.h"
#include "utils/FileUtil.h"
#include <iostream>
using namespace std;
using namespace cv;

float GetMax(const cv::Mat &mat)
{
    double min_val, max_val;
    Point a, b;
    cv::minMaxLoc(mat, &min_val, &max_val, &a, &b);
    return max_val;
}
cv::Mat cColorDepthFixer::FixColorDepth(const cv::Mat &img_rgb_, const cv::Mat &img_depth_input_, float alpha /*= 1*/)
{
    // color.convertTo(color, CV_32FC3);
    // depth.convertTo(depth, CV_32FC1);
    cv::Mat img_rgb = img_rgb_;
    cv::Mat img_depth_input = img_depth_input_;
    if (img_rgb.type() == CV_8UC3)
    {
        img_rgb.convertTo(img_rgb, CV_32FC3);
    }
    if (img_depth_input.type() == CV_8UC1)
    {
        img_depth_input.convertTo(img_depth_input, CV_32FC1);
    }
    // std::cout << "img_depth_input type " << cOpencvUtil::type2str(img_depth_input.type()) << std::endl;
    // std::cout << "img_rgb type " << cOpencvUtil::type2str(img_rgb.type()) << std::endl;
    // exit(1);
    CV_Assert(img_depth_input.type() == CV_32FC1);
    CV_Assert(img_rgb.type() == CV_32FC3);
    CV_Assert(img_rgb.cols == img_depth_input.cols);
    CV_Assert(img_rgb.rows == img_depth_input.rows);

    // std::cout << "img depth input =" << img_depth_input << std::endl;
    // std::cout << "img_rgb = " << img_rgb << std::endl;
    // exit(1);
    // show current result, and then print

    Mat img_isnoise = (img_depth_input == 0) / 255;

    // imshow("img_isnoise", img_isnoise);

    float max_img_abs_depth = GetMax(img_depth_input);
    // std::cout << "imgIsNoise " << img_isnoise << std::endl;
    // std::cout << "max img abs = depth " << max_img_abs_depth << std::endl;

    Mat img_depth = img_depth_input / max_img_abs_depth;
    // std::cout << "img depth = " << img_depth << std::endl;
    CV_Assert(GetMax(img_depth) <= 1);

    int H = img_depth.rows;
    int W = img_depth.cols;
    int num_pixels = H * W;
    tMatrixXi indsM(H, W);
    for (int i = 0; i < W; i++)
    {
        indsM.col(i).setLinSpaced(H * i, H * (i + 1) - 1);
    }
    // std::cout << indsM << std::endl;
    // std::cout << H << std::endl;

    Mat known_val_mask = (img_depth_input != 0) / 255;
    // ! gray image is slightly bigger than python code (0.01)
    cv::Mat gray_img(img_rgb.rows, img_rgb.cols, CV_32FC1);
    cv::cvtColor(img_rgb, gray_img, CV_BGR2GRAY);
    gray_img /= 255;

    // std::cout << "H = " << H << std::endl;
    // std::cout << "W = " << W << std::endl;
    // std::cout << "num_pix = " << num_pixels << std::endl;
    // std::cout << "indsM = " << indsM << std::endl;
    // exit(1);
    // std::cout << "knownValMask = " << known_val_mask << std::endl;
    int win_rad = 1;
    int len_ = 0;
    int abs_img_ndx = 0;
    int len_window = (2 * win_rad + 1) * (2 * win_rad + 1);
    int len_zeros = num_pixels * len_window;
    // std::cout << "gray_img = " << gray_img << std::endl;
    // std::cout << "win_rad = " << win_rad << std::endl;
    // std::cout << "len_ = " << len_ << std::endl;
    // std::cout << "abs_img_ndx = " << abs_img_ndx << std::endl;
    // std::cout << "len_window = " << len_window << std::endl;
    // std::cout << "len_zeros = " << len_zeros << std::endl;
    // exit(1);
    tVectorXi cols = -tVectorXi::Ones(len_zeros),
              rows = -tVectorXi::Ones(len_zeros);
    tVectorXf vals = -tVectorXf::Ones(len_zeros),
              gvals = -tVectorXf::Ones(len_window);
    tVectorXf tmp_vec;

    for (int j = 0; j < W; j++)
    {
        printf("\r %d/%d", j, W);
        for (int i = 0; i < H; i++)
        {
            int n_win = 0;
            for (int ii = std::max(0, i - win_rad); ii < std::min(i + win_rad + 1, H); ii++)
            {
                for (int jj = std::max(0, j - win_rad); jj < std::min(j + win_rad + 1, W); jj++)
                {
                    if (ii == i && jj == j)
                        continue;
                    else
                    {
                        rows[len_] = abs_img_ndx;
                        cols[len_] = indsM(ii, jj);
                        // printf("set row[%d] = %d, set col[%d] = %d\n", len_, abs_img_ndx, len_, cols[len_]);
                        // std::cout << "n_win = " << n_win << std::endl;
                        // std::cout << "gvals size = " << gvals.size() << std::endl;
                        // std::cout << "ii = " << ii << std::endl;
                        // std::cout << "jj = " << jj << std::endl;
                        // std::cout << "gray img shape = " << gray_img.rows
                        //           << " " << gray_img.cols << std::endl;
                        gvals[n_win] = gray_img.at<float>(ii, jj);

                        len_ += 1;
                        n_win += 1;
                    }
                }
            }
            // printf("-----------current point (%d,%d)------------\n", i, j);
            // std::cout << "n_win = " << n_win << std::endl;
            // std::cout << "rows = " << rows.transpose() << std::endl;
            // std::cout << "cols = " << cols.transpose() << std::endl;
            // std::cout << "gvals = " << gvals.transpose() << std::endl;
            float cur_val = gray_img.at<float>(i, j);
            gvals[n_win] = cur_val;

            // again
            tmp_vec = gvals.segment(0, n_win + 1) - gvals.segment(0, n_win + 1).mean() * tVectorXf::Ones(n_win + 1);
            tmp_vec = tmp_vec.array().pow(2);
            float c_var = tmp_vec.mean();
            float c_sig = c_var * 0.6;

            // again
            tmp_vec = gvals.segment(0, n_win) - cur_val * tVectorXf::Ones(n_win);
            tmp_vec = tmp_vec.array().pow(2);
            float mgv = tmp_vec.minCoeff();
            if (c_sig < -mgv / std::log(1e-2))
                c_sig = -mgv / std::log(1e-2);
            if (c_sig < 2e-6)
            {
                c_sig = 2e-6;
            }

            // printf("cur val %f\n", cur_val);
            // std::cout << "gvals " << gvals.transpose() << std::endl;
            // printf("c_var %f\n", c_var);
            // printf("csig %f\n", c_sig);
            // printf("mgv %f\n", mgv);
            // exit(1);

            tmp_vec = (gvals.segment(0, n_win) - cur_val * tVectorXf::Ones(n_win)).array().pow(2);
            tmp_vec = -1 * tmp_vec / c_sig;
            gvals.segment(0, n_win) = tmp_vec.array().exp();
            gvals.segment(0, n_win) /= gvals.segment(0, n_win).sum();

            vals.segment(len_ - n_win, n_win) = -gvals.segment(0, n_win);

            // self reference
            rows[len_] = abs_img_ndx;
            cols[len_] = abs_img_ndx;
            vals[len_] = 1;

            len_ += 1;
            abs_img_ndx += 1;
            // std::cout << "final rows = " << rows.transpose() << std::endl;
            // std::cout << "final cols = " << cols.transpose() << std::endl;
            // std::cout << "final gvals = " << gvals.transpose() << std::endl;
            // exit(1);
        }
    }
    // exit(1);
    // std::cout << "ultimate rows = " << rows.transpose() << std::endl;
    // std::cout << "ultimate cols = " << cols.transpose() << std::endl;
    // std::cout << "ultimate gvals = " << gvals.transpose() << std::endl;
    // std::cout << "ultimate vals = " << vals.transpose() << std::endl;
    // exit(1);
    vals = vals.segment(0, len_).eval();
    cols = cols.segment(0, len_).eval();
    rows = rows.segment(0, len_).eval();

    // std::cout << "ultimate vals = " << vals.transpose() << std::endl;
    // std::cout << "ultimate rows = " << rows.transpose() << std::endl;
    // std::cout << "ultimate cols = " << cols.transpose() << std::endl;
    // exit(1);
    // std::cout << "vals size " << vals.size() << std::endl;
    // std::cout << "cols size " << cols.size() << std::endl;
    // std::cout << "rows size " << rows.size() << std::endl;
    Eigen::SparseMatrix<float> A(num_pixels, num_pixels);
    // printf("begin to fill A\n");
    A.reserve(vals.size());
    {
        tEigenArr<Eigen::Triplet<float>> tris(0);
        OMP_PARALLEL
        {
            tEigenArr<Eigen::Triplet<float>> tris_private;
#pragma omp for nowait
            for (int i = 0; i < vals.size(); i++)
            {
                // printf("\r %d/%d", i, vals.size());
                tris_private.push_back(Eigen::Triplet<float>(rows[i], cols[i], vals[i]));
                // printf("G row %d col %d value %f\n", rows[i],
                //        cols[i], vals[i]);
            }
#pragma omp critical
            tris.insert(tris.begin(), tris_private.begin(), tris_private.end());
        }
        A.setFromTriplets(tris.begin(), tris.end());
    }
    // std::cout << A << std::endl;
    // exit(1);
    rows.resize(num_pixels);
    cols.resize(num_pixels);
    rows.setLinSpaced(0, num_pixels);
    cols.setLinSpaced(0, num_pixels);
    // std::cout << "rows = " << rows.transpose() << std::endl;
    // exit(1);
    const cv::Mat &tmp_mat = known_val_mask * alpha;
    const tMatrixXf &eigen_mat = cOpencvUtil::ConvertOpencvToEigenMatFloat(tmp_mat);
    vals.resize(num_pixels);
    memcpy(vals.data(), eigen_mat.data(), sizeof(float) * vals.size());
    // for (int i = 0; i < vals.size(); i++)
    // {
    //     printf("idx %d row %d col %d val %f\n", i, rows[i], cols[i], vals[i]);
    // }
    // exit(1);
    // printf("begin to fill G\n");
    Eigen::SparseMatrix<float> G(num_pixels, num_pixels);
    G.reserve(vals.size());
    {
        tEigenArr<Eigen::Triplet<float>> tris(0);
        OMP_PARALLEL
        {
            tEigenArr<Eigen::Triplet<float>> tris_private;
#pragma omp for nowait
            for (int i = 0; i < vals.size(); i++)
            {
                // printf("\r %d/%d", i, vals.size());
                tris_private.push_back(Eigen::Triplet<float>(rows[i], cols[i], vals[i]));
                // printf("G row %d col %d value %f\n", rows[i],
                //        cols[i], vals[i]);
            }
#pragma omp critical
            tris.insert(tris.begin(), tris_private.begin(), tris_private.end());
        }
        G.setFromTriplets(tris.begin(), tris.end());
    }
    // exit(1);
    // for (int k = 0; k < A.outerSize(); ++k)
    //     for (Eigen::SparseMatrix<float>::InnerIterator it(A, k); it; ++it)
    //     {
    //         printf("A row %d col %d value %f\n", it.row(),
    //                it.col(), it.value());
    //     }
    // for (int k = 0; k < G.outerSize(); ++k)
    //     for (Eigen::SparseMatrix<float>::InnerIterator it(G, k); it; ++it)
    //     {
    //         printf("G row %d col %d value %f\n", it.row(),
    //                it.col(), it.value());
    //     }

    // std::cout << "A = " << A << std::endl;
    A = A + G;

    tMatrixXf &tmp_mat_img_depth = cOpencvUtil::ConvertOpencvToEigenMatFloat(img_depth);

    Eigen::Map<Eigen::VectorXf> img_depth_vec(tmp_mat_img_depth.data(), int(tmp_mat_img_depth.cols() * tmp_mat_img_depth.rows()));
    tVectorXf b = img_depth_vec.cwiseProduct(
        vals);
    // std::cout << "b = " << b.transpose() << std::endl;
    // exit(1);
    // begin to print A and b
    // for (int k = 0; k < A.outerSize(); ++k)
    //     for (Eigen::SparseMatrix<float>::InnerIterator it(A, k); it; ++it)
    //     {
    //         // it.value();
    //         printf("row %d col %d value %f\n", it.row(),
    //                it.col(), it.value());
    //         // ;           // row index
    //         // ;           // col index (here it is equal to k)
    //         // it.index(); // inner index, here it is equal to it.row()
    //     }
    // exit(1);
    // std::cout << "begin to solve\n";
    // cTimeUtil::Begin("solve");
    Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
    // Eigen::LDLT<Eigen::SparseMatrix<float>> solver;
    // Eigen::BiCGSTAB<Eigen::SparseMatrix<float>> solver;
    solver.compute(A);
    tVectorXf solved = solver.solve(b);
    // cTimeUtil::End("solve");
    // std::cout << "solve done\n";
    // std::cout << "solved = " << solved.transpose() << std::endl;
    // exit(1);
    tMatrixXf new_val(H, W);
    memcpy(new_val.data(), solved.data(), new_val.size() * sizeof(float));
    new_val.transposeInPlace();
    // std::cout << "new vals = " << new_val.transpose() << std::endl;
    // exit(1);
    // done

    tMatrixXf denoised_depth_image = new_val * max_img_abs_depth;
    cv::Mat output = cOpencvUtil::ConvertEigenMatFloatToOpencv(denoised_depth_image.transpose());
    // std::cout
    //     << "output convert = " << output << std::endl;
    known_val_mask = 1 - 1 * known_val_mask;
    known_val_mask.convertTo(known_val_mask, CV_32FC1);
    // std::cout << "known_val_mask type = " << cOpencvUtil::type2str(known_val_mask.type()) << std::endl;
    // std::cout << "output type = " << cOpencvUtil::type2str(output.type()) << std::endl;
    // std::cout << "output size = " << cOpencvUtil::GetOpencvMatSize(output).transpose() << std::endl;
    // std::cout << "mask size = " << cOpencvUtil::GetOpencvMatSize(known_val_mask).transpose() << std::endl;
    // std::cout << "H = " << H << std::endl;
    // std::cout << "W = " << W << std::endl;
    // output *= ;

    output = output.mul(known_val_mask);
    // std::cout
    //     << "output second = " << output << std::endl;
    // exit(1);
    output += img_depth_input;
    // std::cout
    //     << "output = " << output << std::endl;
    // printf("return output\n");

    return output;
    // auto output = (1 - known_val_mask) * denoised_depth_image + img_depth_input;
    // return output;
}

void cColorDepthFixer::FixColorDepth(std::string color_path, std::string depth_path, float alpha /* = 1*/)
{
    SIM_ASSERT(cFileUtil::ExistsFile(color_path));
    SIM_ASSERT(cFileUtil::ExistsFile(depth_path));
    cv::Mat color = cv::imread(color_path, cv::IMREAD_COLOR);
    cv::Mat depth = cv::imread(depth_path, cv::IMREAD_GRAYSCALE);

    auto fixed_depth = cColorDepthFixer::FixColorDepth(
        color, depth, alpha);
    color.convertTo(color, CV_8UC3);
    fixed_depth.convertTo(fixed_depth, CV_8UC1);
    // fixed_depth.convertTo(fixed_depth, CV_32FC1);
    cv::imshow("color", color);
    // cv::waitKey(0);
    cv::imshow("raw_depth", depth);
    // cv::waitKey(0);
    cv::imshow("fixed_depth", fixed_depth);
    cv::imwrite("fixed_depth.png", fixed_depth);
    cv::waitKey(0);
}