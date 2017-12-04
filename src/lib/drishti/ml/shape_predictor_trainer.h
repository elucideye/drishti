// Copyright (C) 2014  Davis E. King (davis@dlib.net)
// Copyright (C) 2015-2017  David Hirvonen
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef __drishti_ml_shape_predictor_trainer_h__
#define __drishti_ml_shape_predictor_trainer_h__

#include "drishti/ml/shape_predictor.h"

// clang-format off
#if !DRISHTI_BUILD_MIN_SIZE
#  include <dlib/serialize.h>
#  include <dlib/console_progress_indicator.h>
#  include <dlib/threads.h>
#endif
// clang-format on

#if !DRISHTI_BUILD_MIN_SIZE

DRISHTI_ML_NAMESPACE_BEGIN

class shape_predictor_trainer
{
    /*!
        This thing really only works with unsigned char or rgb_pixel images (since we assume the threshold
        should be in the range [-128,128]).
    !*/
public:
    shape_predictor_trainer()
    {
        _cascade_depth = 10;
        _tree_depth = 4;
        _num_trees_per_cascade_level = 500;
        _nu = 0.1;
        _oversampling_amount = 20;
        _feature_pool_size = 400;
        _lambda = 0.1;
        _num_test_splits = 20;
        _feature_pool_region_padding = 0;
        _verbose = false;
        _num_threads = 0;
    }

    unsigned long get_cascade_depth() const
    {
        return _cascade_depth;
    }

    void set_cascade_depth(
        unsigned long depth)
    {
        DLIB_CASSERT(depth > 0,
            "\t void shape_predictor_trainer::set_cascade_depth()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t depth:  " << depth);

        _cascade_depth = depth;
    }

    unsigned long get_tree_depth() const
    {
        return _tree_depth;
    }

    void set_tree_depth(
        unsigned long depth)
    {
        DLIB_CASSERT(depth > 0,
            "\t void shape_predictor_trainer::set_tree_depth()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t depth:  " << depth);

        _tree_depth = depth;
    }

    unsigned long get_num_trees_per_cascade_level() const
    {
        return _num_trees_per_cascade_level;
    }

    void set_num_trees_per_cascade_level(
        unsigned long num)
    {
        DLIB_CASSERT(num > 0,
            "\t void shape_predictor_trainer::set_num_trees_per_cascade_level()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t num:  " << num);
        _num_trees_per_cascade_level = num;
    }

    double get_nu() const
    {
        return _nu;
    }
    void set_nu(
        double nu)
    {
        DLIB_CASSERT(0 < nu && nu <= 1,
            "\t void shape_predictor_trainer::set_nu()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t nu:  " << nu);

        _nu = nu;
    }

    std::string get_random_seed() const
    {
        return rnd.get_seed();
    }
    void set_random_seed(
        const std::string& seed)
    {
        rnd.set_seed(seed);
    }

    unsigned long get_oversampling_amount() const
    {
        return _oversampling_amount;
    }
    void set_oversampling_amount(
        unsigned long amount)
    {
        DLIB_CASSERT(amount > 0,
            "\t void shape_predictor_trainer::set_oversampling_amount()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t amount: " << amount);

        _oversampling_amount = amount;
    }

    unsigned long get_feature_pool_size() const
    {
        return _feature_pool_size;
    }
    void set_feature_pool_size(
        unsigned long size)
    {
        DLIB_CASSERT(size > 1,
            "\t void shape_predictor_trainer::set_feature_pool_size()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t size: " << size);

        _feature_pool_size = size;
    }

    double get_lambda() const
    {
        return _lambda;
    }
    void set_lambda(
        double lambda)
    {
        DLIB_CASSERT(lambda > 0,
            "\t void shape_predictor_trainer::set_lambda()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t lambda: " << lambda);

        _lambda = lambda;
    }

    unsigned long get_num_test_splits() const
    {
        return _num_test_splits;
    }
    void set_num_test_splits(
        unsigned long num)
    {
        DLIB_CASSERT(num > 0,
            "\t void shape_predictor_trainer::set_num_test_splits()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t num: " << num);

        _num_test_splits = num;
    }

    double get_feature_pool_region_padding() const
    {
        return _feature_pool_region_padding;
    }
    void set_feature_pool_region_padding(
        double padding)
    {
        _feature_pool_region_padding = padding;
    }

    void be_verbose()
    {
        _verbose = true;
    }

    void be_quiet()
    {
        _verbose = false;
    }

    unsigned long get_num_threads() const
    {
        return _num_threads;
    }

    void set_num_threads(unsigned long num)
    {
        _num_threads = num;
    }

    void set_dimensions(const std::vector<int>& dimensions) { _dimensions = dimensions; }
    const std::vector<int>& get_dimensions() const { return _dimensions; }

    void set_ellipse_count(int ellipse_count) { _ellipse_count = ellipse_count; }
    int get_ellipse_count() const { return _ellipse_count; }

    void set_do_npd(bool do_npd) { _do_npd = do_npd; }
    bool get_do_npd() const { return _do_npd; }

    void set_do_affine(bool do_affine) { _do_affine = do_affine; }
    bool get_do_affine() const { return _do_affine; }

    void set_do_line_indexed(bool do_line_indexed) { _do_line_indexed = do_line_indexed; }
    bool get_do_line_indexed() const { return _do_line_indexed; }

    void set_roi(const dlib::drectangle& roi) { _roi = roi; }
    const dlib::drectangle& get_roi() const { return _roi; }

    static void copyShape(const float* ptr, int n, fshape& shape, fshape& shape_full)
    {
        shape_full.set_size(n, 1);
        memcpy(&shape_full(0), ptr, sizeof(float) * n);
        shape = shape_full;
    }

    template <typename image_array>
    shape_predictor train(
        const image_array& images,
        const std::vector<std::vector<dlib::full_object_detection>>& objects,
        const std::map<int,float> &weights = {}) const
    {
        using namespace impl;
        DLIB_CASSERT(images.size() == objects.size() && images.size() > 0,
            "\t shape_predictor shape_predictor_trainer::train()"
                << "\n\t Invalid inputs were given to this function. "
                << "\n\t images.size():  " << images.size()
                << "\n\t objects.size(): " << objects.size());
        // make sure the objects agree on the number of parts and that there is at
        // least one full_object_detection.
        unsigned long num_parts = 0;
        for (unsigned long i = 0; i < objects.size(); ++i)
        {
            for (unsigned long j = 0; j < objects[i].size(); ++j)
            {
                if (num_parts == 0)
                {
                    num_parts = objects[i][j].num_parts();
                    DLIB_CASSERT(objects[i][j].num_parts() != 0,
                        "\t shape_predictor shape_predictor_trainer::train()"
                            << "\n\t You can't give objects that don't have any parts to the trainer.");
                }
                else
                {
                    DLIB_CASSERT(objects[i][j].num_parts() == num_parts,
                        "\t shape_predictor shape_predictor_trainer::train()"
                            << "\n\t All the objects must agree on the number of parts. "
                            << "\n\t objects[" << i << "][" << j << "].num_parts(): " << objects[i][j].num_parts()
                            << "\n\t num_parts:  " << num_parts);
                }
            }
        }
        DLIB_CASSERT(num_parts != 0,
            "\t shape_predictor shape_predictor_trainer::train()"
                << "\n\t You must give at least one full_object_detection if you want to train a shape model and it must have parts.");

        rnd.set_seed(get_random_seed());

        dlib::thread_pool tp(_num_threads > 1 ? _num_threads : 0);

        DLIB_CASSERT(!(_ellipse_count % 2), "\t currently limited to ellipse pairs"); // point representation limitations

        // For ellipse only, pose indexing is performed via homography:
        bool is_ellipse_only = (_ellipse_count * 5) == (num_parts * 2); // really only works for ellipse pairs

        std::vector<training_sample> samples;
        const fshape initial_shape = populate_training_sample_shapes(objects, samples, _ellipse_count);

        std::vector<PointVecf> pixel_coordinates;
        std::vector<std::vector<InterpolatedFeature>> interpolated_features;

        if (_do_line_indexed)
        {
            randomly_sample_pixel_coordinates_between_features(interpolated_features, pixel_coordinates, initial_shape, _ellipse_count, _roi);
        }
        else
        {
            pixel_coordinates = randomly_sample_pixel_coordinates(initial_shape, _ellipse_count, _roi);
        }

        // PCA for shape space regression:
        const int num_dim = int(initial_shape.size());
        bool do_pca = _dimensions.size() > 0;
        StandardizedPCAPtr pca;
        if (do_pca)
        {
            pca = compute_pca(samples, num_dim, _dimensions, weights);
        }

        unsigned long trees_fit_so_far = 0;
        dlib::console_progress_indicator pbar(get_cascade_depth() * get_num_trees_per_cascade_level());
        if (_verbose)
        {
            std::cout << "Fitting trees..." << std::endl;
        }

        std::vector<std::vector<impl::regression_tree>> forests(get_cascade_depth());
        // Now start doing the actual training by filling in the forests
        for (unsigned long cascade = 0; cascade < get_cascade_depth(); ++cascade)
        {
            int current_pca_dim = do_pca ? _dimensions[cascade] : num_dim;

            // We proceed to fit models coarse-to-fine in shape space, increasing dimensionality at each cascade:
            if (do_pca)
            {
                initialize_shape_space_models(samples, current_pca_dim, pca);
            }

            // Each cascade uses a different set of pixels for its features.  We compute
            // their representations relative to the initial shape first.
            std::vector<unsigned short> anchor_idx;
            PointVecf deltas;

            // For line indexed features we don't need the delta
            if (is_ellipse_only)
            {
                // TODO: Just use homography corresopnding to first ellipse
                assert(false);
            }
            else if (!_do_line_indexed)
            {
                create_shape_relative_encoding(initial_shape, pixel_coordinates[cascade], anchor_idx, deltas, _ellipse_count);
            }

            // First compute the feature_pixel_values for each training sample at this
            // level of the cascade.

            parallel_for(tp, 0, samples.size(), [&](unsigned long i) {
                auto& s = samples[i];
                auto& is = initial_shape;
                const auto& cs = s.current_shape;
                const auto& image = images[s.image_idx];

                if (is_ellipse_only)
                {
                    // TODO: Just use homography corresponding to first ellipse:
                    assert(false);
                }
                else if (_do_line_indexed)
                {
                    extract_feature_pixel_values(image, s.rect, cs, interpolated_features[cascade], s.feature_pixel_values);
                }
                else
                {
                    extract_feature_pixel_values(image, s.rect, cs, is, anchor_idx, deltas, s.feature_pixel_values, _ellipse_count, _do_affine);
                }
            },
                1);

            // Now start building the trees at this cascade level.
            for (unsigned long i = 0; i < get_num_trees_per_cascade_level(); ++i)
            {
                forests[cascade].push_back(make_regression_tree(tp, samples, pixel_coordinates[cascade], _do_npd, do_pca));
                if (_verbose)
                {
                    ++trees_fit_so_far;
                    pbar.print_status(trees_fit_so_far);
                }
            }

            if (do_pca)
            {
                update_shape_space_models(samples, current_pca_dim);
            }
        }

        if (_verbose)
        {
            std::cout << "Training complete                          " << std::endl;
        }

        if (interpolated_features.size())
        {
            return shape_predictor(initial_shape, forests, interpolated_features, pca, _do_npd, _do_affine, _ellipse_count);
        }
        else
        {
            return shape_predictor(initial_shape, forests, pixel_coordinates, pca, _do_npd, _do_affine, _ellipse_count);
        }
    }

private:
    static fshape object_to_shape(
        const dlib::full_object_detection& obj,
        int ellipse_count = 0)
    {
        fshape shape(obj.num_parts() * 2 - (5 * ellipse_count));
        const dlib::point_transform_affine tform_from_img = impl::normalizing_tform(obj.get_rect());

        int end = int(obj.num_parts()) - (ellipse_count * 5);
        for (unsigned long i = 0; i < end; ++i)
        {
            dlib::vector<float, 2> p = tform_from_img(obj.part(i));
            shape(2 * i + 0) = p.x();
            shape(2 * i + 1) = p.y();
        }

        // Normalize ellipse in homogeneous coordinates:
        for (int i = end, j = 2 * end; i < (end + ellipse_count * 5); i += 5)
        {
            cv::RotatedRect e, e2;
            e.center.x = obj.part(i + 0).x();
            e.center.y = obj.part(i + 1).x();
            e.size.width = obj.part(i + 2).x();
            e.size.height = obj.part(i + 3).x();
            e.angle = obj.part(i + 4).x();

            const auto& m = tform_from_img.get_m();
            const auto& b = tform_from_img.get_b();
            cv::Matx33f H(m(0, 0), m(0, 1), b(0), m(1, 0), m(1, 1), b(1), 0, 0, 1);

            std::vector<float> phi = geometry::ellipseToPhi(H * e);
            for (int k = 0; k < phi.size(); k++, j++)
            {
                shape(j) = phi[k];
            }
        }

        return shape;
    }

    struct training_sample
    {
        /*!

        CONVENTION
            - feature_pixel_values.size() == get_feature_pool_size()
            - feature_pixel_values[j] == the value of the j-th feature pool
              pixel when you look it up relative to the shape in current_shape.

            - target_shape == The truth shape.  Stays constant during the whole
              training process.
            - rect == the position of the object in the image_idx-th image.  All shape
              coordinates are coded relative to this rectangle.

            - diff_shape == temporary value for holding difference between current
              shape and target shape
         
            - trailing _ indicates shape space projection:
        !*/

        unsigned long image_idx;
        dlib::rectangle rect;

        fshape target_shape, target_shape_, target_shape_full_;
        fshape current_shape, current_shape_, current_shape_full_;
        fshape diff_shape;
        std::vector<float> feature_pixel_values;

        void swap(training_sample& item)
        {
            std::swap(image_idx, item.image_idx);
            std::swap(rect, item.rect);
            target_shape.swap(item.target_shape);
            current_shape.swap(item.current_shape);
            feature_pixel_values.swap(item.feature_pixel_values);

            target_shape_.swap(item.target_shape_);
            current_shape_.swap(item.current_shape_);

            target_shape_full_.swap(item.target_shape_full_);
            current_shape_full_.swap(item.current_shape_full_);

            diff_shape.swap(item.diff_shape);
        }
    };

    void update_shape_space_models(std::vector<training_sample>& samples, int current_pca_dim) const
    {
        auto current_range = dlib::range(0, current_pca_dim - 1);
        for (auto& s : samples)
        {
            // Update our shape space model:
            s.current_shape_full_ = 0;
            dlib::set_rowm(s.current_shape_full_, current_range) = s.current_shape_;
        }
    }

    void initialize_shape_space_models(std::vector<training_sample>& samples, int current_pca_dim, StandardizedPCAPtr& pca) const
    {
        auto current_range = dlib::range(0, current_pca_dim - 1);
        for (auto& s : samples)
        {
            s.current_shape_ = dlib::rowm(s.current_shape_full_, current_range);
            s.target_shape_ = dlib::rowm(s.target_shape_full_, current_range);

            CV_Assert(s.current_shape_.size() == current_pca_dim);
            CV_Assert(s.target_shape_.size() == current_pca_dim);

            // Back project from (potentially coarse) shape space to the full euclidean model:
            cv::Mat1f cs1_(1, int(s.current_shape_.size()), &s.current_shape_(0));
            cv::Mat1f cs1 = pca->backProject(cs1_);
            memcpy(&s.current_shape(0), cs1.ptr<float>(), sizeof(float) * cs1.cols);

#if DRISHTI_DLIB_DO_NUMERIC_DEBUG
            for (auto& v : s.current_shape_)
            {
                CV_Assert(!std::isnan(v));
            }
            for (auto& v : s.target_shape_)
            {
                CV_Assert(!std::isnan(v));
            }
            for (auto& v : s.current_shape)
            {
                CV_Assert(!std::isnan(v));
            }
            for (auto& v : s.current_shape_full_)
            {
                CV_Assert(!std::isnan(v));
            }
            if (first)
            {
                std::cout << dlib::trans(s.current_shape) << std::endl;
                std::cout << dlib::trans(s.current_shape_full_) << std::endl;
                std::cout << dlib::trans(s.current_shape_) << std::endl;
                std::cout << dlib::trans(s.target_shape_) << std::endl;
                first = false;
            }
#endif
        }
    }

    using IntVec = std::vector<int>;
    using SampleVec = std::vector<training_sample>;

    StandardizedPCAPtr compute_pca(SampleVec& samples, int num_dim, const IntVec& dimensions, const std::map<int,float> &sparse_weights={}) const
    {
        using namespace impl;
        CV_Assert(int(dimensions.size()) == get_cascade_depth());

        cv::Mat1f ts, cs, ts_, cs_;
        int max_pca_dim = *std::max_element(dimensions.begin(), dimensions.end());

        std::vector<cv::Mat1f> tss, css, extra;
        for (int i = 0; i < samples.size(); i++)
        {
            tss.push_back(cv::Mat1f(1, num_dim, &samples[i].target_shape(0)));
            css.push_back(cv::Mat1f(1, num_dim, &samples[i].current_shape(0)));
        }

        cv::vconcat(tss, ts);
        cv::vconcat(css, cs);

        // Use the full set of target and current (randomized) poses for PCA
        cv::Mat full;
        cv::vconcat(ts, cs, full);

        cv::Mat weights(1, full.cols, CV_32F, cv::Scalar::all(1.0));
        for(const auto &w : sparse_weights)
        {
            // Here we assume the weights are only applied to the leading 2D points (not trailing ellipse)
            weights.at<float>((w.first * 2)+0) = w.second;
            weights.at<float>((w.first * 2)+1) = w.second;
        }
        
        std::cout << "Using weights: " << weights << std::endl;

        auto pca = std::make_shared<StandardizedPCA>();
        
        cv::Mat full_;
        pca->compute(full, full_, max_pca_dim, weights);

        //cv::Mat full_;
        //pca->compute(ts, full_, max_pca_dim);

        ts_ = pca->project(ts);
        cs_ = pca->project(cs);

        for (int i = 0; i < samples.size(); i++)
        {
            copyShape(cs_.ptr<float>(i), cs_.cols, samples[i].current_shape_, samples[i].current_shape_full_);
            copyShape(ts_.ptr<float>(i), ts_.cols, samples[i].target_shape_, samples[i].target_shape_full_);
        }

        return pca;
    }

    impl::regression_tree make_regression_tree(
        dlib::thread_pool& tp,
        std::vector<training_sample>& samples,
        const PointVecf& pixel_coordinates,
        bool do_npd = false,
        bool do_pca = false) const
    {
        using namespace impl;
        std::deque<std::pair<unsigned long, unsigned long>> parts;
        parts.push_back(std::make_pair(0, (unsigned long)samples.size()));

        impl::regression_tree tree;

        // walk the tree in breadth first order
        const unsigned long num_split_nodes = static_cast<unsigned long>(std::pow(2.0, (double)get_tree_depth()) - 1);
        std::vector<fshape> sums(num_split_nodes * 2 + 1);

        if (tp.num_threads_in_pool() > 1)
        {
            // Here we need to calculate shape differences and store sum of differences into sums[0]
            // to make it. I am splitting samples into blocks, each block will be processed by
            // separate thread, and the sum of differences of each block is stored into separate
            // place in block_sums

            const unsigned long num_workers = std::max(1UL, tp.num_threads_in_pool());
            const unsigned long num = samples.size();
            const unsigned long block_size = std::max(1UL, (num + num_workers - 1) / num_workers);
            std::vector<dlib::matrix<float, 0, 1>> block_sums(num_workers);

            parallel_for(tp, 0, num_workers, [&](unsigned long block) {
                const unsigned long block_begin = block * block_size;
                const unsigned long block_end = std::min(num, block_begin + block_size);
                for (unsigned long i = block_begin; i < block_end; ++i)
                {
                    if (do_pca) // #if DO_PCA_INTERNAL
                    {
                        samples[i].diff_shape = samples[i].target_shape_ - samples[i].current_shape_;
                    }
                    else
                    {
                        samples[i].diff_shape = samples[i].target_shape - samples[i].current_shape;
                    }
                    block_sums[block] += samples[i].diff_shape;
                }
            },
                1);

            // now calculate the total result from separate blocks
            for (unsigned long i = 0; i < block_sums.size(); ++i)
            {
                sums[0] += block_sums[i];
            }
        }
        else
        {
            // synchronous:
            for (unsigned long i = 0; i < samples.size(); ++i)
            {
                if (do_pca) // #if DO_PCA_INTERNAL
                {
                    samples[i].diff_shape = samples[i].target_shape_ - samples[i].current_shape_;
                }
                else
                {
                    samples[i].diff_shape = samples[i].target_shape - samples[i].current_shape;
                }
                sums[0] += samples[i].diff_shape;
            }
        }

        for (unsigned long i = 0; i < num_split_nodes; ++i)
        {
            std::pair<unsigned long, unsigned long> range = parts.front();
            parts.pop_front();

            auto& sumsL = sums[left_child(i)];
            auto& sumsR = sums[right_child(i)];
            impl::split_feature split = generate_split(tp, samples, range.first, range.second, pixel_coordinates, sums[i], sumsL, sumsR, do_npd, do_pca);
            tree.splits.push_back(split);
            const unsigned long mid = partition_samples(split, samples, range.first, range.second, do_npd);

            parts.push_back(std::make_pair(range.first, mid));
            parts.push_back(std::make_pair(mid, range.second));
        }

        // Now all the parts contain the ranges for the leaves so we can use them to
        // compute the average leaf values.
        tree.leaf_values.resize(parts.size());
        for (unsigned long i = 0; i < parts.size(); ++i)
        {
            if (parts[i].second != parts[i].first)
            {
                tree.leaf_values[i] = sums[num_split_nodes + i] * get_nu() / (parts[i].second - parts[i].first);
            }
            else
            {
                if (do_pca)
                {
                    tree.leaf_values[i] = zeros_matrix(samples[0].target_shape_);
                }
                else
                {
                    tree.leaf_values[i] = zeros_matrix(samples[0].target_shape);
                }
            }

            // now adjust the current shape based on these predictions
            parallel_for(tp, parts[i].first, parts[i].second, [&](unsigned long j) {
                if (do_pca)
                {
                    samples[j].current_shape_ += tree.leaf_values[i];
                }
                else
                {
                    samples[j].current_shape += tree.leaf_values[i];
                }
            },
                1);
        }

        return tree;
    }

    impl::split_feature randomly_generate_split_feature(
        const PointVecf& pixel_coordinates,
        bool do_npd = false) const
    {
        const double lambda = get_lambda();
        impl::split_feature feat;
        double accept_prob;
        do
        {
            feat.idx1 = rnd.get_random_16bit_number() % get_feature_pool_size();
            feat.idx2 = rnd.get_random_16bit_number() % get_feature_pool_size();
            const double dist = length(pixel_coordinates[feat.idx1] - pixel_coordinates[feat.idx2]);
            accept_prob = std::exp(-dist / lambda);
        } while (feat.idx1 == feat.idx2 || !(accept_prob > rnd.get_random_double()));

        if (do_npd)
        {
            feat.thresh = ((rnd.get_random_double() * 2.0) - 1.0);
        }
        else
        {
            feat.thresh = (rnd.get_random_double() * 256 - 128) / 2.0;
        }

        return feat;
    }

    impl::split_feature generate_split(
        dlib::thread_pool& tp,
        const std::vector<training_sample>& samples,
        unsigned long begin,
        unsigned long end,
        const PointVecf& pixel_coordinates,
        const fshape& sum,
        fshape& left_sum,
        fshape& right_sum,
        bool do_npd = false,
        bool do_pca = false

        ) const
    {
        // generate a bunch of random splits and test them and return the best one.

        const unsigned long num_test_splits = get_num_test_splits();

        // sample the random features we test in this function
        std::vector<impl::split_feature> feats;
        feats.reserve(num_test_splits);
        for (unsigned long i = 0; i < num_test_splits; ++i)
        {
            feats.push_back(randomly_generate_split_feature(pixel_coordinates, do_npd));
        }

        std::vector<fshape> left_sums(num_test_splits);
        std::vector<unsigned long> left_cnt(num_test_splits);

        const unsigned long num_workers = std::max(1UL, tp.num_threads_in_pool());
        const unsigned long block_size = std::max(1UL, (num_test_splits + num_workers - 1) / num_workers);

        // now compute the sums of vectors that go left for each feature
        parallel_for(tp, 0, num_workers, [&](unsigned long block) {
            const unsigned long block_begin = block * block_size;
            const unsigned long block_end = std::min(block_begin + block_size, num_test_splits);

            for (unsigned long j = begin; j < end; ++j)
            {
                for (unsigned long i = block_begin; i < block_end; ++i)
                {
                    const auto& values1 = samples[j].feature_pixel_values[feats[i].idx1];
                    const auto& values2 = samples[j].feature_pixel_values[feats[i].idx2];
                    if (do_npd)
                    {
                        if (compute_npd(values1, values2) > feats[i].thresh)
                        {
                            left_sums[i] += samples[j].diff_shape;
                            ++left_cnt[i];
                        }
                    }
                    else
                    {
                        if ((static_cast<float>(values1) - static_cast<float>(values2)) > feats[i].thresh)
                        {
                            left_sums[i] += samples[j].diff_shape;
                            ++left_cnt[i];
                        }
                    }
                }
            }
        },
            1);

        // now figure out which feature is the best
        double best_score = -1;
        unsigned long best_feat = 0;
        dlib::matrix<float, 0, 1> temp;
        for (unsigned long i = 0; i < num_test_splits; ++i)
        {
            // check how well the feature splits the space.
            double score = 0;
            unsigned long right_cnt = end - begin - left_cnt[i];
            if (left_cnt[i] != 0 && right_cnt != 0)
            {
                temp = sum - left_sums[i];
                score = dot(left_sums[i], left_sums[i]) / left_cnt[i] + dot(temp, temp) / right_cnt;
                if (score > best_score)
                {
                    best_score = score;
                    best_feat = i;
                }
            }
        }

        left_sums[best_feat].swap(left_sum);
        if (left_sum.size() != 0)
        {
            right_sum = sum - left_sum;
        }
        else
        {
            right_sum = sum;
            left_sum = zeros_matrix(sum);
        }
        return feats[best_feat];
    }

    unsigned long partition_samples(
        const impl::split_feature& split,
        std::vector<training_sample>& samples,
        unsigned long begin,
        unsigned long end,
        bool do_npd = false) const
    {
        // splits samples based on split (sorta like in quick sort) and returns the mid
        // point.  make sure you return the mid in a way compatible with how we walk
        // through the tree.

        unsigned long i = begin;

        if (do_npd)
        {
            for (unsigned long j = begin; j < end; ++j)
            {
                if (compute_npd(samples[j].feature_pixel_values[split.idx1], samples[j].feature_pixel_values[split.idx2]) > split.thresh)
                {
                    samples[i].swap(samples[j]);
                    ++i;
                }
            }
        }
        else
        {
            for (unsigned long j = begin; j < end; ++j)
            {
                if (samples[j].feature_pixel_values[split.idx1] - samples[j].feature_pixel_values[split.idx2] > split.thresh)
                {
                    samples[i].swap(samples[j]);
                    ++i;
                }
            }
        }
        return i;
    }

    fshape populate_training_sample_shapes(
        const std::vector<std::vector<dlib::full_object_detection>>& objects,
        std::vector<training_sample>& samples,
        int ellipse_count = 0) const
    {
        samples.clear();
        fshape mean_shape;
        long count = 0;
        // first fill out the target shapes
        for (unsigned long i = 0; i < objects.size(); ++i)
        {
            for (unsigned long j = 0; j < objects[i].size(); ++j)
            {
                training_sample sample;
                sample.image_idx = i;
                sample.rect = objects[i][j].get_rect();
                sample.target_shape = object_to_shape(objects[i][j], ellipse_count);
                for (unsigned long itr = 0; itr < get_oversampling_amount(); ++itr)
                {
                    samples.push_back(sample);
                }
                mean_shape += sample.target_shape;
                ++count;
            }
        }

        mean_shape /= count;

        // now go pick random initial shapes
        for (unsigned long i = 0; i < samples.size(); ++i)
        {
            if ((i % get_oversampling_amount()) == 0)
            {
                // The mean shape is what we really use as an initial shape so always
                // include it in the training set as an example starting shape.
                samples[i].current_shape = mean_shape;
            }
            else
            {
                // Pick a random convex combination of two of the target shapes and use
                // that as the initial shape for this sample.
                const unsigned long rand_idx1 = rnd.get_random_32bit_number() % samples.size();
                const unsigned long rand_idx2 = rnd.get_random_32bit_number() % samples.size();
                const double alpha = rnd.get_random_double();
                samples[i].current_shape = alpha * samples[rand_idx1].target_shape + (1.0 - alpha) * samples[rand_idx2].target_shape;
            }
        }

        return mean_shape;
    }

    // ================ line features =====================
    void randomly_sample_pixel_coordinates_between_features(
        const fshape& initial_shape,
        std::vector<InterpolatedFeature>& locations,
        PointVecf& pixel_coordinates,
        double padding,
        const dlib::drectangle& roi = { 0.f, 0.f, 0.f, 0.f } // exclusion roi
        ) const
    /*!
     ensures
     - #pixel_coordinates.size() == get_feature_pool_size()
     - for all valid i:
     - location[i] == a line indexed feature point (between two features)
     !*/
    {
        int point_length = int(initial_shape.size()) / 2;
        locations.resize(get_feature_pool_size());
        pixel_coordinates.resize(get_feature_pool_size());
        for (unsigned long i = 0; i < get_feature_pool_size(); ++i)
        {
            fpoint p;
            do
            {
                // Avoid points inside the roi:
                locations[i].f1 = rnd.get_random_32bit_number() % point_length;
                locations[i].f2 = rnd.get_random_32bit_number() % point_length;
                locations[i].f3 = rnd.get_random_32bit_number() % point_length;

                // https://github.com/ChrisYang/TIFfacealignment/blob/15c60270e99f6d0ab45b5ec4fb971ba47a921abe/dlib-18.16/dlib/image_processing/shape_predictor_TIF.h#L1061-L1072
                // The TIF code uses two random numbers with range [0 ... 0.5]
                //
                //const double alpha = rnd.get_random_double() * 0.5;
                //const double beta = rnd.get_random_double() * 0.5;
                
                // This scheme will allow sampling along either ray's exteme points
                //
                const double alpha = rnd.get_random_double();
                const double beta = (1.0 - alpha) * rnd.get_random_double();
                
                locations[i].w12 = static_cast<float>(alpha);
                locations[i].w13 = static_cast<float>(beta);
                
                pixel_coordinates[i] = impl::interpolate_feature_point(locations[i], initial_shape);
            } while (roi.contains(pixel_coordinates[i]));
        }
    }

    void randomly_sample_pixel_coordinates_between_features(
        std::vector<std::vector<InterpolatedFeature>>& indexed_features,
        std::vector<PointVecf>& pixel_coordinates,
        const fshape& initial_shape,
        int ellipse_count = 0,
        const dlib::drectangle& roi = { 0.f, 0.f, 0.f, 0.f }) const
    {
        const double padding = get_feature_pool_region_padding();
        // Figure out the bounds on the object shapes.  We will sample uniformly
        // from this box.

        // matrix range op
        int ellipseLength = (5 * ellipse_count);
        dlib::matrix<float> temp_ = dlib::subm(initial_shape, dlib::range(0, initial_shape.size() - 1 - ellipseLength), dlib::range(0, 1));

        dlib::matrix<float> temp = reshape(temp_, temp_.size() / 2, 2);
        pixel_coordinates.resize(get_cascade_depth());
        indexed_features.resize(get_cascade_depth());
        for (unsigned long i = 0; i < get_cascade_depth(); ++i)
        {
            randomly_sample_pixel_coordinates_between_features(temp_, indexed_features[i], pixel_coordinates[i], padding, roi);
        }
    }

    void randomly_sample_pixel_coordinates(
        PointVecf& pixel_coordinates,
        const double min_x,
        const double min_y,
        const double max_x,
        const double max_y,
        const dlib::drectangle& roi = { 0.f, 0.f, 0.f, 0.f }) const
    /*!
        ensures
            - #pixel_coordinates.size() == get_feature_pool_size()
            - for all valid i:
                - pixel_coordinates[i] == a point in the box defined by the min/max x/y arguments.
    !*/
    {
        pixel_coordinates.resize(get_feature_pool_size());
        for (unsigned long i = 0; i < get_feature_pool_size(); ++i)
        {
            do
            {
                // Avoid points inside the roi:
                pixel_coordinates[i].x() = rnd.get_random_double() * (max_x - min_x) + min_x;
                pixel_coordinates[i].y() = rnd.get_random_double() * (max_y - min_y) + min_y;
            } while (roi.contains(pixel_coordinates[i]));
        }
    }

    std::vector<PointVecf> randomly_sample_pixel_coordinates(
        const fshape& initial_shape,
        int ellipse_count = 0,
        const dlib::drectangle& roi = { 0.f, 0.f, 0.f, 0.f }) const
    {
        const double padding = get_feature_pool_region_padding();
        // Figure out the bounds on the object shapes.  We will sample uniformly
        // from this box.

        int ellipseLength = (5 * ellipse_count);
        // matrix range op
        dlib::matrix<float> temp_ = dlib::subm(initial_shape, dlib::range(0, initial_shape.size() - 1 - ellipseLength), dlib::range(0, 1));

        dlib::matrix<float> temp = reshape(temp_, temp_.size() / 2, 2);
        const double min_x = min(colm(temp, 0)) - padding;
        const double min_y = min(colm(temp, 1)) - padding;
        const double max_x = max(colm(temp, 0)) + padding;
        const double max_y = max(colm(temp, 1)) + padding;

        std::vector<PointVecf> pixel_coordinates;
        pixel_coordinates.resize(get_cascade_depth());
        for (unsigned long i = 0; i < get_cascade_depth(); ++i)
        {
            randomly_sample_pixel_coordinates(pixel_coordinates[i], min_x, min_y, max_x, max_y, roi);
        }
        return pixel_coordinates;
    }

    mutable dlib::rand rnd;

    unsigned long _cascade_depth;
    unsigned long _tree_depth;
    unsigned long _num_trees_per_cascade_level;
    double _nu;
    unsigned long _oversampling_amount;
    unsigned long _feature_pool_size;
    double _lambda;
    unsigned long _num_test_splits;
    double _feature_pool_region_padding;
    bool _verbose;
    unsigned long _num_threads;

    // new parameters
    std::vector<int> _dimensions;
    int _ellipse_count = 0; /* trailing N * 5 params represent ellipses and need different normalization */
    bool _do_npd = false;
    bool _do_affine = false;
    bool _do_line_indexed = false;
    dlib::drectangle _roi = { 0.f, 0.f, 0.f, 0.f };

    // experimental
    std::map<int, impl::recipe> _recipe_for_cascade_level;
};

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

template <
    typename image_array>
double test_shape_predictor(
    const shape_predictor& sp,
    const image_array& images,
    const std::vector<std::vector<dlib::full_object_detection>>& objects,
    const std::vector<std::vector<double>>& scales,
    int ellipse_count = 0)
{
// make sure requires clause is not broken
#ifdef ENABLE_ASSERTS
    DLIB_CASSERT(images.size() == objects.size(),
        "\t double test_shape_predictor()"
            << "\n\t Invalid inputs were given to this function. "
            << "\n\t images.size():  " << images.size()
            << "\n\t objects.size(): " << objects.size());
    for (unsigned long i = 0; i < objects.size(); ++i)
    {
        for (unsigned long j = 0; j < objects[i].size(); ++j)
        {
            DLIB_CASSERT(objects[i][j].num_parts() == sp.num_parts(),
                "\t double test_shape_predictor()"
                    << "\n\t Invalid inputs were given to this function. "
                    << "\n\t objects[" << i << "][" << j << "].num_parts(): " << objects[i][j].num_parts()
                    << "\n\t sp.num_parts(): " << sp.num_parts());
        }
        if (scales.size() != 0)
        {
            DLIB_CASSERT(objects[i].size() == scales[i].size(),
                "\t double test_shape_predictor()"
                    << "\n\t Invalid inputs were given to this function. "
                    << "\n\t objects[" << i << "].size(): " << objects[i].size()
                    << "\n\t scales[" << i << "].size(): " << scales[i].size());
        }
    }
#endif

    dlib::running_stats<double> rs;
    for (unsigned long i = 0; i < objects.size(); ++i)
    {
        for (unsigned long j = 0; j < objects[i].size(); ++j)
        {
            // Just use a scale of 1 (i.e. no scale at all) if the caller didn't supply
            // any scales.
            const double scale = scales.size() == 0 ? 1 : scales[i][j];

            dlib::full_object_detection det = sp(images[i], objects[i][j].get_rect());

            for (unsigned long k = 0; k < det.num_parts(); ++k)
            {
                double score = length(det.part(k) - objects[i][j].part(k)) / scale;
                rs.add(score);
            }
        }
    }
    return rs.mean();
}

template <
    typename image_array>
double test_shape_predictor(
    const shape_predictor& sp,
    const image_array& images,
    const std::vector<std::vector<dlib::full_object_detection>>& objects)
{
    std::vector<std::vector<double>> no_scales;
    return test_shape_predictor(sp, images, objects, no_scales);
}

DRISHTI_ML_NAMESPACE_END

#endif // !DRISHTI_BUILD_MIN_SIZE

#endif // __drishti_ml_shape_predictor_trainer_h__
