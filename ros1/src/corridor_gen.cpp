#include <corridor_gen.h>

using namespace CorridorGen;
CorridorGenerator::CorridorGenerator(
    double resolution, double clearance, int max_sample, 
    double ceiling, double floor, double goal_pt_margin,
    bool is_sparse,
    std::vector<Eigen::Vector4d> no_flight_zone)
    : resolution_(resolution), clearance_(clearance), 
    max_sample_(max_sample), ceiling_(ceiling), 
    floor_(floor), goal_pt_margin_(goal_pt_margin),
    is_sparse_(is_sparse),
    no_flight_zone_(no_flight_zone)
{
    octree_.deleteTree();
    octree_.setResolution(resolution_);
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    one_third_ = 1.0 / 3.0;
    // normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    // normal_rand_(gen_);
    closeness_threshold_ = 0.5;
}

void CorridorGenerator::updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_cloud)
{
    cloud_ = new_cloud;
    octree_.deleteTree();
    octree_.setInputCloud(new_cloud);
    octree_.addPointsFromInputCloud();
}

void CorridorGenerator::updateGlobalPath(const std::vector<Eigen::Vector3d> &path)
{
    guide_path_.clear();
    // std::cout << "original path " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    guide_path_ = path;               // copy assignment
    initial_position_ = path.front(); // front() and back() return read-only reference
    goal_position_ = path.back();
    std::reverse(guide_path_.begin(), guide_path_.end());
}

bool CorridorGenerator::generateCorridorAlongPath(const std::vector<Eigen::Vector3d> &path)
{
    push_back_count = 0;
    flight_corridor_.clear(); // clear flight corridor generated from previous iteration
    waypoint_list_.clear();   // clear waypoints initialized from previous iteration
    sample_direction_.clear();
    updateGlobalPath(path);
    waypoint_list_.push_back(initial_position_);
    Corridor current_corridor = GenerateOneSphere(initial_position_);
    push_back_count++;
    Corridor previous_corridor = current_corridor;
    current_corridor.second = 1;
    flight_corridor_.emplace_back(current_corridor);
    int max_iter = guide_path_.size();
    int iter_count = 0;
    std::cout << "before while true" << std::endl;
    std::cout << "initial_position_ " << initial_position_.transpose() << std::endl;
    // todo: implement time check to break from loop
    bool guide_point_adjusted = false;
    Eigen::Vector3d sample_origin;
    int stagnant_count = 0;
    while (true)
    {
        iter_count++;
        if (iter_count > 200)
        {
            std::cout << "OVER MAX_ITER FAIL to generate corridor" << std::endl;
            return false;
        }
        if (!guide_point_adjusted)
        {
            stagnant_count = 0;
            local_guide_point_ = getGuidePoint(guide_path_, current_corridor);
            sample_origin = local_guide_point_;
            closeness_threshold_ = 0.5;
        }
        // std::cout << "local_guide_point_ " << local_guide_point_.transpose() << std::endl;
        // current_corridor = batchSample(local_guide_point_, current_corridor);
        current_corridor = uniformBatchSample(local_guide_point_, current_corridor, sample_origin, guide_point_adjusted);
        double distance_between_corridor_center = (current_corridor.first - previous_corridor.first).norm();
        std::cout << "distance_between_corridor_center " << distance_between_corridor_center << std::endl;
        std::cout << " current_corridor " << current_corridor.first.transpose() << std::endl;

        // check if the corridor did not move much i.e. clutter place
        // need to adjust guide point according?
        // guide point will affect the sample direction
        // AND sample origin
        // after adjustment, we will need to use the same sample origin?
        // if (distance_between_corridor_center < 0.5) // this check can be done in batchSample?

        if (corridorTooClose(current_corridor))
        {
            guide_point_adjusted = true;
            current_corridor = previous_corridor; // is this needed?
            stagnant_count++;
            closeness_threshold_ = current_corridor.second;
            if (guide_path_.size() < 2)
            {
                std::cout << "TOO STAGNANT and guide_path empty FAIL to generate corridor" << std::endl;
                return false;
            }
            if (stagnant_count > 3)
            {
                closeness_threshold_ = closeness_threshold_ - stagnant_count * 0.1;
                if (closeness_threshold_ < 0.2)
                {
                    closeness_threshold_ = 0.2;
                }
                // prevent popping an empty guide_path list
                // if (guide_path_.size() < 2)
                // {
                //     std::cout << "TOO STAGNANT and guide_path empty FAIL to generate corridor" << std::endl;
                //     break;
                // }

                std::cout << "TOO STAGNANT adjusting guide point" << std::endl;
                // break;
                stagnant_count = 0;
                guide_path_.pop_back();
                local_guide_point_ = guide_path_.back();
                // sample_origin = local_guide_point_;
            }

            else
            {
                guide_path_.pop_back();
                local_guide_point_ = guide_path_.back();
                // sample_origin = local_guide_point_;
                continue;
            }

            // std::cout << "skipped" << std::endl;

            // prevent popping an empty guide_path list
            // if (guide_path_.size() < 2)
            // {
            //     std::cout << "guide_path empty fail to generate corridor" << std::endl;
            //     break;
            // }

            // guide_path_.pop_back();
            // local_guide_point_ = guide_path_.back();
            // guide_point_adjusted = true;
            // current_corridor = previous_corridor;
            continue;
        }
        else
        {
            guide_point_adjusted = false;
        }

        // current_corridor = directionalSample(local_guide_point_, current_corridor);
        // current_corridor = GenerateOneSphere(local_guide_point_);
        Eigen::Vector3d intermediate_waypoint = getMidPointBetweenCorridors(previous_corridor, current_corridor);
        previous_corridor = current_corridor;

        // std::cout << "current_corridor centre " << current_corridor.first.transpose() << " radius " << current_corridor.second << std::endl;

        flight_corridor_.emplace_back(current_corridor);
        push_back_count++;
        waypoint_list_.emplace_back(intermediate_waypoint);

        if (pointInCorridor(goal_position_, current_corridor))
        // if (pointNearCorridor(goal_position_, current_corridor))
        {
            std::cout << "Corridors Generated" << std::endl;
            waypoint_list_.emplace_back(goal_position_);
            // std::cout << "waypt size is " << waypoint_list_.size() << std::endl;
            // std::cout << "number of corridor " << flight_corridor_.size() << std::endl;
            // std::cout << "pushed back times " << push_back_count << std::endl;
            return true;
        }
    }
}

Corridor CorridorGenerator::GenerateOneSphere(const Eigen::Vector3d &pos)
{
    // lock guard to prevent cloud from changing?
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchPoint;
    searchPoint.x = pos.x();
    searchPoint.y = pos.y();
    searchPoint.z = pos.z();

    // consider using radiusSearch?
    if (octree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // std::cout << "Found neighbor" << std::endl;
        double radius = sqrt(double(pointNKNSquaredDistance[0])) - clearance_;
        Eigen::Vector3d nearestPt;

        nearestPt.x() = (*cloud_)[pointIdxNKNSearch[0]].x;
        nearestPt.y() = (*cloud_)[pointIdxNKNSearch[0]].y;
        nearestPt.z() = (*cloud_)[pointIdxNKNSearch[0]].z;

        // std::cout << "nearestPt is " << nearestPt.transpose() << std::endl;

        if (radius > searchPoint.z)
        {
            radius = abs(searchPoint.z);
        }

        return Corridor(pos, radius);
    }
    else
    {
        std::cout << "cloud empty?" << std::endl;
        return Corridor(pos, 10.0);
    }
}

auto CorridorGenerator::GenerateOneSphereVerbose(const Eigen::Vector3d &pos)
{
    // lock guard to prevent cloud from changing?
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchPoint;
    searchPoint.x = pos.x();
    searchPoint.y = pos.y();
    searchPoint.z = pos.z();
    Eigen::Vector3d nearestPt;
    // consider using radiusSearch?
    if (octree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // std::cout << "Found neighbor" << std::endl;
        double radius = sqrt(double(pointNKNSquaredDistance[0])) - clearance_;

        nearestPt.x() = (*cloud_)[pointIdxNKNSearch[0]].x;
        nearestPt.y() = (*cloud_)[pointIdxNKNSearch[0]].y;
        nearestPt.z() = (*cloud_)[pointIdxNKNSearch[0]].z;

        // std::cout << "nearestPt is " << nearestPt.transpose() << std::endl;

        if (radius > searchPoint.z)
        {
            radius = abs(searchPoint.z);
        }

        // return Corridor(pos, radius);
        return std::make_pair(Corridor(pos, radius), nearestPt);
    }
    else
    {
        nearestPt << 50, 50, 50;
        std::cout << "cloud empty?" << std::endl;
        // return Corridor(pos, 10.0);
        return std::make_pair(Corridor(pos, 10.0), nearestPt);
    }
}

Eigen::Vector3d CorridorGenerator::getGuidePoint(std::vector<Eigen::Vector3d> &guide_path, const Corridor &input_corridor)
{
    auto [center, radius] = input_corridor;

    // std::cout << "before popping " << std::endl;

    // for (auto point : guide_path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    
    while ((center - guide_path.back()).norm() < radius && 
        guide_path.size() > 1)
    {
        guide_path.pop_back();
        // std::cout << "popped " << std::endl;
    }

    Eigen::Vector3d out;
    if (is_sparse_)
    {
        Eigen::Vector3d sample_direction = 
            (guide_path.back() - center).normalized();
        out = 
            sample_direction * radius + center;
    }
    else
        out = guide_path.back();

    // std::cout << "after popping " << std::endl;

    // for (auto point : guide_path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    return out;
}

bool CorridorGenerator::pointInCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
    auto [center, radius] = corridor;
    return ((point - center).norm()) < radius;
}

bool CorridorGenerator::pointNearCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
    auto [center, radius] = corridor;
    return ((point - center).norm() - goal_pt_margin_) < radius;
}

Corridor CorridorGenerator::batchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
    auto [previous_center, radius] = input_corridor;
    Eigen::Vector3d sample_x_vector = previous_center - guide_point;
    std::cout << "previous_center " << previous_center.transpose() << std::endl;
    std::cout << "guide_point " << guide_point.transpose() << std::endl;
    Eigen::Vector3d sample_x_direction = sample_x_vector / sample_x_vector.norm();
    Eigen::Vector3d global_x_direction(1, 0, 0);
    double angle_between = acos(sample_x_direction.dot(global_x_direction));
    Eigen::Vector3d rotation_axis;
    Eigen::Matrix3d rotation_matrix;

    double x_var = one_third_ * sample_x_vector.norm() / 5;
    double y_var = 2 * x_var;
    double z_var = y_var;
    double x_sd = sqrt(x_var);
    double y_sd = sqrt(y_var);
    double z_sd = y_sd;
    int sample_num = 0;

    if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
    {
        rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    }

    else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
    {
        rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    }

    else
    {
        rotation_axis = global_x_direction.cross(sample_x_direction);
        rotation_axis = rotation_axis / rotation_axis.norm();
        rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
    }

    // rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);

    // x_normal_rand_ = std::normal_distribution<double>(guide_point.x(), x_sd);
    // y_normal_rand_ = std::normal_distribution<double>(guide_point.y(), y_sd);
    // z_normal_rand_ = std::normal_distribution<double>(guide_point.y(), z_sd);

    x_normal_rand_ = std::normal_distribution<double>(0, x_sd);
    y_normal_rand_ = std::normal_distribution<double>(0, y_sd);
    z_normal_rand_ = std::normal_distribution<double>(0, x_sd);
    std::cout << "one_third_ " << one_third_ << std::endl;
    std::cout << "sample_x_vector.norm() " << sample_x_vector.norm() << std::endl;
    std::cout << "x_var " << x_var << std::endl;
    std::cout << "y_var " << y_var << std::endl;
    std::cout << "z_var " << z_var << std::endl;
    std::cout << "x_sd " << x_sd << std::endl;
    std::cout << "y_sd " << y_sd << std::endl;
    std::cout << "z_sd " << z_sd << std::endl;

    std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
    corridor_pool_.swap(empty_pool);

    while (sample_num < max_sample_)
    {
        sample_num++;
        double x_coord = x_normal_rand_(gen_);
        double y_coord = y_normal_rand_(gen_);
        double z_coord = z_normal_rand_(gen_);

        // Eigen::Vector3d candidate_pt(0.0, y_coord, z_coord);
        Eigen::Vector3d candidate_pt(x_coord, y_coord, z_coord);
        // needs to transform the candidate_pt to align the axes
        // std::cout << "before transformation " << candidate_pt.transpose() << std::endl;
        // candidate_pt = rotation_matrix.transpose() * candidate_pt + guide_point;
        candidate_pt = rotation_matrix * candidate_pt + guide_point;
        // std::cout << "candidate_pt " << candidate_pt.transpose() << std::endl;

        // if (double(candidate_pt.z()) > ceiling_ || double(candidate_pt.z()) < floor_)
        // {
        //     std::cout << "skipped" << std::endl;
        //     continue;
        // }

        CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
        candidiate_corridor->geometry = GenerateOneSphere(candidate_pt);
        // skip the sample if it is too small. 0.25 as in the smallest gap of the environment should be
        std::cout << "candidiate_corridor->geometry.second " << candidiate_corridor->geometry.second << std::endl;
        if (candidiate_corridor->geometry.second < 0.25)
        {
            std::cout << "too small" << std::endl;
            continue;
        }
        // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
        candidiate_corridor->updateScore(input_corridor, Eigen::Vector2d(1.0, 1.0));
        corridor_pool_.push(candidiate_corridor);
    }
    std::cout << "size " << corridor_pool_.size() << std::endl;
    if (corridor_pool_.size() == 0)
    {
        return input_corridor;
    }

    auto ret = corridor_pool_.top();
    return ret->geometry;
}

Corridor CorridorGenerator::directionalSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
    Corridor new_corridor;
    auto [pre_center, pre_radius] = input_corridor;
    auto ret = GenerateOneSphereVerbose(guide_point);
    Corridor guide_pt_corridor = ret.first;
    Eigen::Vector3d obstacle = ret.second;
    auto [center, radius] = guide_pt_corridor;
    double desired_radius = 0.5;
    double resolution = 0.01;
    std::vector<Eigen::Vector3d> sample_point_bucket;
    if (radius >= desired_radius)
    {
        return guide_pt_corridor;
    }

    else
    {
        // std::cout << " desired_radius " << desired_radius << std::endl;
        // std::cout << " radius " << radius << std::endl;
        if (desired_radius - radius < resolution)
        {
            return guide_pt_corridor;
        }
        Eigen::Vector3d pre2guide = guide_point - pre_center;
        pre2guide = pre2guide / pre2guide.norm();

        Eigen::Vector3d obs2guide = guide_point - obstacle;
        obs2guide = obs2guide / obs2guide.norm();

        Eigen::Vector3d sample_direction = pre2guide + obs2guide;
        sample_direction = sample_direction / sample_direction.norm();

        Eigen::Vector3d sample_dir_coord = sample_direction + guide_point;
        sample_direction_.push_back(std::make_pair(guide_point, sample_dir_coord));

        Eigen::Vector3d global_x_direction(1, 0, 0);
        double angle_between = acos(sample_direction.dot(global_x_direction));
        Eigen::Vector3d rotation_axis;
        Eigen::Matrix3d rotation_matrix;

        std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
        corridor_pool_.swap(empty_pool);

        if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
        {
            rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        }

        else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
        {
            rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        }

        else
        {
            rotation_axis = global_x_direction.cross(sample_direction);
            rotation_axis = rotation_axis / rotation_axis.norm();
            rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
        }
        int count = 0;
        for (double i = resolution; i < (desired_radius - radius); i += resolution)
        // for (double i = resolution; i < 1; i += resolution)
        {
            count++;
            Eigen::Vector3d candidate_point;
            candidate_point << i, 0.0, 0.0;
            std::cout << " candidate_point " << candidate_point.transpose() << std::endl;
            candidate_point = rotation_matrix * candidate_point + guide_point;

            CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
            candidiate_corridor->geometry = GenerateOneSphere(candidate_point);
            // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
            candidiate_corridor->updateScore(input_corridor, Eigen::Vector2d(1.0, 20.0));
            corridor_pool_.push(candidiate_corridor);
            // sample_point_bucket.push_back(candidate_point);
        }

        std::cout << "walked steps: " << count << std::endl;

        auto ret = corridor_pool_.top();
        return ret->geometry;
    }
}

Corridor CorridorGenerator::uniformBatchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor, const Eigen::Vector3d &sample_around, bool guide_point_adjusted)
{
    bool sample_in_clutter;
    auto [previous_center, pre_radius] = input_corridor;

    Eigen::Vector3d sample_origin;
    if (guide_point_adjusted)
    {
        sample_in_clutter = true;
        Eigen::Vector3d sample_direction = guide_point - previous_center;
        sample_direction = sample_direction / sample_direction.norm();
        sample_origin = sample_direction * pre_radius + previous_center;
    }
    else
    {
        sample_in_clutter = false;
        sample_origin = sample_around;
    }

    // auto [pre_center, pre_radius] = input_corridor;
    // if guide_point is adjusted i.e. it will lead to early return
    // because there will be no overlap between the previous and new corridors
    // auto ret = GenerateOneSphereVerbose(guide_point);

    //
    auto ret = GenerateOneSphereVerbose(sample_origin);

    std::cout << "sample_origin " << sample_origin.transpose() << std::endl;

    Corridor guide_pt_corridor = ret.first;
    Eigen::Vector3d obstacle = ret.second;
    auto [center, radius] = guide_pt_corridor;
    double desired_radius = 1;
    double resolution = 0.1;
    std::vector<Eigen::Vector3d> sample_point_bucket;

    // Following check is needed if guide_point is adjusted
    // Also handle Special case when the sample_origin (guide point) is very close to the obstacle
    // we will continue for batchsample
    double distance_between = (previous_center - center).norm();
    if (distance_between > (pre_radius + radius) && (radius > 0.25) && (!guide_point_adjusted))
    {
        std::cout << "Candidate too far, no overlap " << center.transpose() << std::endl;
        return input_corridor;
    }

    if (distance_between > (pre_radius + radius) && (radius > 0.25) && (guide_point_adjusted))
    {
        std::cout << "After adjustment Candidate too far, no overlap " << center.transpose() << std::endl;
        return input_corridor;
    }

    if (radius >= desired_radius)
    {
        return guide_pt_corridor;
    }

    else
    {
        // informedRRT(guide_point, goal_position_);
        // todo combine logic with if statement
        if (desired_radius - radius < resolution)
        {
            return guide_pt_corridor;
        }
        Eigen::Vector3d sample_to;
        Eigen::Vector3d sample_from;
        sample_from = sample_origin;
        sample_to = previous_center;
        sample_to.z() = sample_from.z(); // to ensure sample direction is along the guide path z plane
        Eigen::Vector3d sample_x_vector = sample_to - guide_point;
        Eigen::Vector3d sample_dir_coord = sample_x_vector + guide_point;
        // sample_direction_.push_back(std::make_pair(guide_point, sample_dir_coord));
        sample_direction_.push_back(std::make_pair(sample_from, sample_to));
        // Eigen::Vector3d sample_x_vector = guide_point - previous_center;
        // Eigen::Vector3d sample_x_vector = previous_center - goal_position_;
        // std::cout << "previous_center " << previous_center.transpose() << std::endl;
        // std::cout << "guide_point " << guide_point.transpose() << std::endl;
        Eigen::Vector3d sample_x_direction = sample_x_vector / sample_x_vector.norm();
        Eigen::Vector3d global_x_direction(1, 0, 0);
        double angle_between = acos(sample_x_direction.dot(global_x_direction));
        Eigen::Vector3d rotation_axis;
        Eigen::Matrix3d rotation_matrix;

        double x_var = one_third_ * sample_x_vector.norm() / 10;
        double y_var = 2 * x_var;
        double z_var = y_var;
        double x_sd = sqrt(x_var);
        double y_sd = sqrt(y_var);
        double z_sd = y_sd;
        int sample_num = 0;

        if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
        {
            rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        }

        else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
        {
            rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        }

        else
        {
            rotation_axis = global_x_direction.cross(sample_x_direction);
            rotation_axis = rotation_axis / rotation_axis.norm();
            rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
        }

        x_normal_rand_ = std::normal_distribution<double>(0, x_sd);
        y_normal_rand_ = std::normal_distribution<double>(0, y_sd);
        z_normal_rand_ = std::normal_distribution<double>(0, z_sd);
        double sample_length = sample_x_vector.norm();
        std::cout << "-sample_length, sample_length " << sample_length << std::endl;
        std::uniform_real_distribution<double> x_uniform_rand_(-sample_length, sample_length);
        std::uniform_real_distribution<double> y_uniform_rand_(-0.5, 0.5);
        std::uniform_real_distribution<double> z_uniform_rand_(-0.2, 0.2);

        std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
        corridor_pool_.swap(empty_pool);

        Eigen::Vector2d weighting;

        while (sample_num < max_sample_)
        {
            flag:

            sample_num++;
            Eigen::Vector3d candidate_pt;
            if (sample_in_clutter)
            {
                std::cout << "sample_in_clutter" << std::endl;

                x_uniform_rand_ = std::uniform_real_distribution<double>(-sample_length, 0);
                y_uniform_rand_ = std::uniform_real_distribution<double>(-sample_length, sample_length);
                z_uniform_rand_ = std::uniform_real_distribution<double>(0, 0);
                // double x_coord = x_normal_rand_(gen_);
                // double y_coord = y_normal_rand_(gen_);
                // double z_coord = z_normal_rand_(gen_);
                double x_coord = x_uniform_rand_(gen_);
                double y_coord = y_uniform_rand_(gen_);
                double z_coord = z_uniform_rand_(gen_);
                candidate_pt << x_coord, y_coord, 0.0;
                weighting << 1.0, 5.0;
            }

            else
            {
                double x_coord = x_normal_rand_(gen_);
                double y_coord = y_normal_rand_(gen_);
                double z_coord = z_normal_rand_(gen_);

                // Eigen::Vector3d candidate_pt(0.0, y_coord, z_coord);
                // Eigen::Vector3d candidate_pt(x_coord, y_coord, z_coord);

                candidate_pt << x_coord, y_coord, z_coord;
                weighting << 2.0, 5.0;
            }

            // needs to transform the candidate_pt to align the axes
            candidate_pt = rotation_matrix * candidate_pt + sample_origin;
            candidate_pt.z() = sample_origin.z();

            for (int i = 0; i < (int)no_flight_zone_.size(); i++)
            {
                // x_min, x_max, y_min, y_max in original frame
                double x_min = no_flight_zone_[i][0], x_max = no_flight_zone_[i][1];
                double y_min = no_flight_zone_[i][2], y_max = no_flight_zone_[i][3];
                
                // Reject point if it is in no fly zone
                if (candidate_pt.x() <= x_max && candidate_pt.x() >= x_min &&
                    candidate_pt.y() <= y_max && candidate_pt.y() >= y_min)
                    goto flag;
            }

            CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
            candidiate_corridor->geometry = GenerateOneSphere(candidate_pt);

            // skip the sample if it is too small 0.5 as in the smallest gap of the environment should be
            if (candidiate_corridor->geometry.second < 0.15 || candidiate_corridor->geometry.second < 0)
            {
                continue;
            }

            if (sample_in_clutter)
            {
                std::cout << "clutter sample radius is " << candidiate_corridor->geometry.second << std::endl;
            }
            // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
            candidiate_corridor->updateScore(input_corridor, weighting);

            if (candidiate_corridor->score == 0)
            {
                continue; // do not add candidate_corrior to pool if there is no overlap
            }
            else
            {
                corridor_pool_.push(candidiate_corridor);
            }
        }
        if (corridor_pool_.size() == 0)
        {
            std::cout << " no candidate found " << std::endl;
            return input_corridor;
        }

        // auto ret = corridor_pool_.top();
        // return ret->geometry;
        return corridor_pool_.top()->geometry;
    }
}

// about return a vector efficiently
// https://stackoverflow.com/questions/28760475/how-to-return-a-class-member-vector-in-c11
const std::vector<Corridor> CorridorGenerator::getCorridor() const
{
    return flight_corridor_;
}

const std::vector<Eigen::Vector3d> CorridorGenerator::getWaypointList() const
{
    return waypoint_list_; // waypoints in sequence from start to end
}

const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> CorridorGenerator::getSampleDirection() const
{
    return sample_direction_;
}

const std::vector<Eigen::Vector3d> CorridorGenerator::getSamplePoint() const
{
    return sample_points_;
}

Eigen::Vector3d CorridorGenerator::getMidPointBetweenCorridors(const Corridor &previous_corridor, const Corridor &current_corridor)
{
    auto [pre_center, pre_radius] = previous_corridor;
    auto [curr_center, curr_radius] = current_corridor;
    // std::cout << "pre_center " << pre_center.transpose() << " pre_radius " << pre_radius << std::endl;
    // std::cout << "curr_center " << curr_center.transpose() << " curr_radius " << curr_radius << std::endl;
    Eigen::Vector3d previous2current;
    previous2current = curr_center - pre_center;
    double distance_between = previous2current.norm();
    previous2current = previous2current / distance_between;
    double distance_to_waypt = pre_radius - (pre_radius + curr_radius - distance_between) * 0.5;
    Eigen::Vector3d waypt = previous2current * distance_to_waypt + pre_center;
    std::cout << "waypt " << waypt.transpose() << std::endl;
    return waypt;
}

bool CorridorGenerator::corridorTooClose(const Corridor &corridor_for_check)
{
    std::cout << "threshold is " << closeness_threshold_ << std::endl;
    if (flight_corridor_.empty())
    {
        return false;
    }

    double closest_distance = 100;
    ;

    auto [query_center, query_radius] = corridor_for_check;

    for (auto corridor : flight_corridor_)
    {
        auto [curr_center, curr_radius] = corridor;

        double distance = (curr_center - query_center).norm();

        if (distance < closest_distance)
        {
            closest_distance = distance;
        }
    }

    if (closest_distance < closeness_threshold_)
    {
        std::cout << "corridor too close, skip" << std::endl;
        std::cout << "threshold is " << closeness_threshold_ << std::endl;
        return true;
    }

    else
    {
        return false;
    }
}