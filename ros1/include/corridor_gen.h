#include <iostream>
#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <algorithm>
#include <random>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CorridorGen
{
    struct CorridorSample;
    using Corridor = std::pair<Eigen::Vector3d, double>;
    using CorridorPtr = std::shared_ptr<CorridorSample>;
    using namespace pcl;

    struct CorridorSample
    {
        Corridor geometry; // position and radius of the spherical corridor sample
        // CorridorPtr parent = nullptr;
        double ego_volume;
        double overlap_volume;
        double score;
        // CorridorSample(CorridorPtr parent) : parent(parent){}

        static constexpr double PI = 3.1415926;
        static constexpr double four_thirds = 1.33333;

        void updateEgoVolume()
        {
            double radius = geometry.second;
            ego_volume = four_thirds * PI * radius * radius * radius;
        }

        // https://mathworld.wolfram.com/Sphere-SphereIntersection.html eq(16)
        void updateOverlapVolume(const Corridor &previous_corridor)
        {
            auto [pre_center, pre_radius] = previous_corridor;
            auto [curr_center, curr_radius] = geometry;
            double distance_between = (pre_center - curr_center).norm();

            if (distance_between > (pre_radius + curr_radius))
            {
                overlap_volume = 0;
            }

            else
            {
                double first = pre_radius + curr_radius - distance_between;
                double second = distance_between * distance_between + 2 * distance_between * (pre_radius + curr_radius) - 3 * (pre_radius * pre_radius + curr_radius * curr_radius) + 6 * pre_radius * curr_radius;
                overlap_volume = PI * first * first * second / 12 / distance_between;
            }
        }

        void updateScore(const Corridor &previous_corridor, Eigen::Vector2d weighting)
        {
            // check if overlap before even calling updateScore?
            updateEgoVolume();
            updateOverlapVolume(previous_corridor);
            // if (overlap_volume < 0.02)
            // {
            //     score = 0;
            // }
            if (overlap_volume == 0)
            {
                score = 0;
            }
            else
            {
                score = Eigen::Vector2d(ego_volume, overlap_volume).transpose() * weighting;
                // std::cout << "current center is " << geometry.first.transpose() << std::endl;
                // std::cout << "current radius is " << geometry.second << std::endl;
                // std::cout << "overlap_volume " << overlap_volume << std::endl;
            }

            // std::cout << " score " << score << " overlap " << overlap_volume * weighting(1) << " ego_volume " << ego_volume * weighting(0) << std::endl;
            // std::cout << " overlap " << overlap_volume << " ego_volume " << ego_volume << std::endl;
        }
    };

    class CorridorComparator
    {
    public:
        bool operator()(CorridorPtr c1, CorridorPtr c2)
        {
            return c1->score < c2->score;
        }
    };

    class CorridorGenerator
    {
    public:
    public: // public member variable
        // flight_corridor_[i].first is the position of i^th spherical corridor centre
        // flight_corridor_[i].second is the radius of i^th spherical corridor
        std::vector<Corridor> flight_corridor_;

    private: // private member variable
        Eigen::Vector3d initial_position_, 
            goal_position_, 
            local_guide_point_;
        double resolution_; // point cloud resolution
        double clearance_;  // drone radius
        double one_third_;
        double ceiling_; // height limit
        double floor_;   // floor limit
        double goal_pt_margin_;
        double closeness_threshold_; // this will follow the radius of corridor in clutter environment

        int max_sample_;

        bool is_sparse_;

        std::vector<Eigen::Vector4d> no_flight_zone_;

        // global guide path
        std::vector<Eigen::Vector3d> guide_path_;
        std::vector<Eigen::Vector3d> waypoint_list_;

        // debug
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> sample_direction_;
        std::vector<Eigen::Vector3d> sample_points_;
        int push_back_count;

        // store the local point cloud
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_ =
            decltype(octree_)(0.1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

        // for sampling
        std::mt19937_64 gen_;
        std::normal_distribution<double> x_normal_rand_;
        std::normal_distribution<double> y_normal_rand_;
        std::normal_distribution<double> z_normal_rand_;

        // priority queue
        std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> corridor_pool_;

        // OctreePointCloudPointVector<PointXYZ> octree_(0.1);
        // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;

    public: // public member function
        CorridorGenerator(
            double resolution, double clearance, int max_sample_, double ceiling, 
            double floor_, double goal_pt_margin, bool is_sparse,
            std::vector<Eigen::Vector4d> no_flight_zone);
        ~CorridorGenerator() = default;

        void updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_cloud); // yes

        // Generate a sphere SFC given a position
        Corridor GenerateOneSphere(const Eigen::Vector3d &pos);
        auto GenerateOneSphereVerbose(const Eigen::Vector3d &pos);

        void updateGlobalPath(const std::vector<Eigen::Vector3d> &path); // yes
        const std::vector<Corridor> getCorridor() const;
        const std::vector<Eigen::Vector3d> getWaypointList() const;
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getSampleDirection() const;
        const std::vector<Eigen::Vector3d> getSamplePoint() const;
        bool generateCorridorAlongPath(const std::vector<Eigen::Vector3d> &path); // wip

    private: // private member function
        Eigen::Vector3d getGuidePoint(std::vector<Eigen::Vector3d> &guide_path, const Corridor &input_corridor);
        bool pointInCorridor(const Eigen::Vector3d &point, const Corridor &corridor);
        bool pointNearCorridor(const Eigen::Vector3d &point, const Corridor &corridor);
        Corridor batchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor);
        Corridor directionalSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor);
        Corridor uniformBatchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor, const Eigen::Vector3d &sample_origin, bool guide_point_adjusted);
        Eigen::Vector3d getMidPointBetweenCorridors(const Corridor &previous_corridor, const Corridor &current_corridor);
        bool corridorTooClose(const Corridor &corridor_for_check);
        void informedRRT(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

        // more functions to implement batchSample(...)
    };

}; // namespace CorridorGenerator