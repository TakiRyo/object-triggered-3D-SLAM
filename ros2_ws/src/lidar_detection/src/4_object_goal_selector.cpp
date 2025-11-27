#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

struct TrackedCluster
{
  float min_x, max_x;
  float min_y, max_y;
  float cx, cy;
  float width, height;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool stable;
};

class ObjectClusterMarker : public rclcpp::Node
{
public:
  ObjectClusterMarker()
  : Node("object_goal_selector")
  {
    // --- Parameters ---
    this->declare_parameter("cluster_distance_threshold", 4.0); // クラスタリング距離（少し広めに設定）
    this->declare_parameter("min_cluster_points", 8); 
    this->declare_parameter("wall_thickness_threshold", 0.3); 
    this->declare_parameter("stability_time", 3.0); 

    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    wall_thickness_threshold_ = this->get_parameter("wall_thickness_threshold").as_double();
    stability_time_ = this->get_parameter("stability_time").as_double();

    // --- Subscribers & Publishers ---
    object_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10,
      std::bind(&ObjectClusterMarker::cloudCallback, this, std::placeholders::_1));

    candidate_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_clusters", 10);
    stable_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/stable_clusters", 10);

    RCLCPP_INFO(this->get_logger(),
      "✅ ObjectGoalSelector started (dist=%.2f, min_pts=%d, wall=%.2f, stable=%.1fs)",
      cluster_distance_threshold_, min_cluster_points_, wall_thickness_threshold_, stability_time_);
  }

private:
  // --- 2つのリスト（短期記憶と長期記憶） ---
  std::vector<TrackedCluster> candidates_;      // 候補 (Yellow)
  std::vector<TrackedCluster> stable_objects_;  // 確定 (Green)

  // --- Helper: Overlap check ---
  bool isOverlap(const TrackedCluster &a, const TrackedCluster &b)
  {
    bool x_overlap = (b.min_x <= a.max_x) && (b.max_x >= a.min_x);
    bool y_overlap = (b.min_y <= a.max_y) && (b.max_y >= a.min_y);
    return x_overlap && y_overlap;
  }

  // --- Helper: Inclusion check ---
  bool isIncluded(const TrackedCluster &a, const TrackedCluster &b)
  {
    bool x_included = (a.min_x >= b.min_x) && (a.max_x <= b.max_x);
    bool y_included = (a.min_y >= b.min_y) && (a.max_y <= b.max_y);
    return x_included || y_included;
  }

  // --- Helper: Merge Logic (サイズは大きい方を維持、時間は更新) ---
  void mergeCluster(TrackedCluster &target, const TrackedCluster &source, rclcpp::Time now)
  {
    target.last_seen = now; // 生存確認

    // サイズ更新：大きい方を採用 (Max Size Lock)
    target.min_x = std::min(target.min_x, source.min_x);
    target.max_x = std::max(target.max_x, source.max_x);
    target.min_y = std::min(target.min_y, source.min_y);
    target.max_y = std::max(target.max_y, source.max_y);
    
    target.width  = std::max(target.width, source.width);
    target.height = std::max(target.height, source.height);

    // 中心位置は最新に更新（ロボットのオドメトリ誤差補正のため）
    target.cx = source.cx;
    target.cy = source.cy;
  }

  // --- Main callback ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    // ==========================================
    // Step 1: Extract points
    // ==========================================
    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return;

    // ==========================================
    // Step 2: Create clusters (distance-based)
    // ==========================================
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i)
    {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      float dist = std::sqrt(dx*dx + dy*dy);

      if (dist > cluster_distance_threshold_) {
        if (current.size() >= static_cast<size_t>(min_cluster_points_))
          clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (current.size() >= static_cast<size_t>(min_cluster_points_))
      clusters.push_back(current);

    // ==========================================
    // Step 3: Convert to "Raw Observations" & Filter Walls
    // ==========================================
    std::vector<TrackedCluster> raw_observations; // 今この瞬間見えているもの
    for (const auto &cluster : clusters)
    {
      float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
      for (const auto &p : cluster) {
        min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
        min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
      }
      float width  = std::max(0.05f, max_x - min_x);
      float height = std::max(0.05f, max_y - min_y);

      // 薄い壁を除去
      if (std::min(width, height) < wall_thickness_threshold_) continue;

      // 生データを作成（IDやStableフラグはまだない）
      raw_observations.push_back({min_x, max_x, min_y, max_y, 
                                  (min_x+max_x)/2.0f, (min_y+max_y)/2.0f, 
                                  width, height, now, now, false});
    }

    // ==========================================
    // Step 4: 照合（マッチング）
    // ==========================================
    for (auto &raw : raw_observations)
    {
      bool matched = false;

      // --- 4-1. 最優先：安定リスト(Green)との照合 ---
      for (auto &stable : stable_objects_)
      {
        if (isIncluded(raw, stable) || isIncluded(stable, raw) || isOverlap(raw, stable))
        {
          // マッチした！ -> 安定クラスターを更新
          mergeCluster(stable, raw, now);
          matched = true;
          break; // このrawデータは吸収されたので終了
        }
      }
      if (matched) continue; // 次のrawデータへ

      // --- 4-2. 次点：候補リスト(Yellow)との照合 ---
      for (auto &cand : candidates_)
      {
        if (isIncluded(raw, cand) || isIncluded(cand, raw) || isOverlap(raw, cand))
        {
          // マッチした！ -> 候補クラスターを更新
          mergeCluster(cand, raw, now);
          matched = true;
          break;
        }
      }
      if (matched) continue;

      // --- 4-3. 新規登録 ---
      // どこにもマッチしなかったので、新しい「候補」として登録
      candidates_.push_back(raw);
    }

    // ==========================================
    // Step 5: 管理（昇格と削除）
    // ==========================================
    
    auto it = candidates_.begin();
    while (it != candidates_.end())
    {
      double age = (now - it->first_seen).seconds();
      double unseen_time = (now - it->last_seen).seconds();

      // A. 昇格 (Promotion): 3秒以上存在したら安定リストへ移動
      if (age > stability_time_) {
        it->stable = true;
        stable_objects_.push_back(*it); // 緑リストへコピー
        it = candidates_.erase(it);     // 黄リストからは削除
        RCLCPP_INFO(this->get_logger(), "🟢 Object Promoted to Stable!");
        continue;
      }

      // B. ノイズ削除: 0.5秒見えなかったら削除
      if (unseen_time > 0.5) {
        it = candidates_.erase(it);
      } else {
        ++it;
      }
    }

    // C. 記憶の維持: stable_objects_ は削除処理を行わない（永続化）
    // （もし遠すぎるものを消したい場合はここにロジックを追加）

    // ==========================================
    // Step 6: Publish
    // ==========================================
    visualization_msgs::msg::MarkerArray candidate_array, stable_array;
    int id_counter = 0;

    // 緑（Stable）のマーカー作成
    for (const auto &c : stable_objects_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = now;
      marker.ns = "stable_clusters";
      marker.id = id_counter++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = c.cx; marker.pose.position.y = c.cy; marker.pose.position.z = 0.1;
      marker.scale.x = c.width; marker.scale.y = c.height; marker.scale.z = 0.1;
      marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.8f;
      stable_array.markers.push_back(marker);
    }

    // 黄（Candidate）のマーカー作成
    for (const auto &c : candidates_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = now;
      marker.ns = "candidate_clusters";
      marker.id = id_counter++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = c.cx; marker.pose.position.y = c.cy; marker.pose.position.z = 0.1;
      marker.scale.x = c.width; marker.scale.y = c.height; marker.scale.z = 0.1;
      marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.6f;
      candidate_array.markers.push_back(marker);
    }

    candidate_pub_->publish(candidate_array);
    stable_pub_->publish(stable_array);
    RCLCPP_INFO(this->get_logger(), "Status: 🟨 %zu Candidates | 🟩 %zu Stable Objects",
                candidates_.size(), stable_objects_.size());
  }

  // --- Members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stable_pub_;
  
  double cluster_distance_threshold_;
  int min_cluster_points_;
  double wall_thickness_threshold_;
  double stability_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectClusterMarker>());
  rclcpp::shutdown();
  return 0;
}