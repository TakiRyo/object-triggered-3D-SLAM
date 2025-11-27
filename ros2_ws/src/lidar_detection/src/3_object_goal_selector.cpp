// choose stable (tracked) object. 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

// --- 1. 瞬間のデータ（LIDARの1フレーム分の情報） ---
struct RawCandidate
{
  float cx, cy;
  float min_x, max_x, min_y, max_y;
  float width, height;
};

// --- 2. 記憶データ（ロボットが覚えている物体リスト） ---
struct TrackedObject
{
  int id;                  // ユニークID
  float cx, cy;            // 現在の中心位置
  float min_x, max_x;      // バウンディングボックス
  float min_y, max_y;
  float width, height;     // ★過去最大のサイズを保持する
  
  rclcpp::Time first_seen; // 初めて見た時間
  rclcpp::Time last_seen;  // 最後に見えた時間
  bool stable;             // 安定しているか（緑色）
};

class ObjectTrackerNode : public rclcpp::Node
{
public:
  ObjectTrackerNode()
  : Node("object_tracker_node"), next_obj_id_(0)
  {
    // --- パラメータ設定 ---
    this->declare_parameter("cluster_distance", 2.5);   // 点同士をまとめる距離
    this->declare_parameter("min_points", 8);          // 最小点数（ノイズ除去）
    this->declare_parameter("stability_time", 3.0);     // 安定するまでの時間
    this->declare_parameter("ghost_dist", 2.0);         // 安定物体の近くのノイズを除去する距離
    this->declare_parameter("memory_time", 5.0);        // 見失っても覚えている時間

    cluster_dist_ = this->get_parameter("cluster_distance").as_double();
    min_pts_ = this->get_parameter("min_points").as_int();
    stable_time_ = this->get_parameter("stability_time").as_double();
    ghost_dist_ = this->get_parameter("ghost_dist").as_double();
    memory_time_ = this->get_parameter("memory_time").as_double();

    // Pub/Sub
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/object_clusters", 10, std::bind(&ObjectTrackerNode::cloudCallback, this, std::placeholders::_1));
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracked_objects", 10);
    
    RCLCPP_INFO(this->get_logger(), "✅ Object Tracker Started");
  }

private:
  // --- メンバ変数 ---
  std::vector<TrackedObject> object_list_; // ★これが「ノート（記憶）」です
  int next_obj_id_;
  
  double cluster_dist_, stable_time_, ghost_dist_, memory_time_;
  int min_pts_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // --- ヘルパー関数: 距離計算 ---
  float getDist(float x1, float y1, float x2, float y2) {
    return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
  }

  // --- ヘルパー関数: 重なりチェック ---
  bool isOverlap(const TrackedObject &o, const RawCandidate &c) {
    // 包含関係も含めてチェック（シンプルなAABB交差判定）
    bool x_over = (c.min_x <= o.max_x) && (c.max_x >= o.min_x);
    bool y_over = (c.min_y <= o.max_y) && (c.max_y >= o.min_y);
    return x_over && y_over;
  }

  // --- メイン処理 ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    // ==========================================
    // STEP 1: LIDARデータから「候補」を抽出
    // ==========================================
    std::vector<RawCandidate> candidates = extractCandidates(msg);

    // ==========================================
    // STEP 2: 「記憶」と「候補」の答え合わせ (Matching)
    // ==========================================
    std::vector<bool> candidate_used(candidates.size(), false);

    for (auto &obj : object_list_)
    {
      int best_idx = -1;
      float best_dist = 100.0; 

      // このオブジェクトに一番近い（または重なっている）候補を探す
      for (size_t i = 0; i < candidates.size(); ++i)
      {
        if (candidate_used[i]) continue;

        float d = getDist(obj.cx, obj.cy, candidates[i].cx, candidates[i].cy);
        bool overlap = isOverlap(obj, candidates[i]);

        // 重なっている、または 0.5m 以内なら「同じ物体」とみなす
        if (overlap || d < 0.5) {
          if (d < best_dist) {
            best_dist = d;
            best_idx = i;
          }
        }
      }

      // マッチしたら情報を更新 (Update)
      if (best_idx != -1) {
        updateObject(obj, candidates[best_idx], now);
        candidate_used[best_idx] = true; 
      }
    }

    // ==========================================
    // STEP 3: 新しい物体の登録 (Create)
    // ==========================================
    for (size_t i = 0; i < candidates.size(); ++i)
    {
      if (!candidate_used[i]) {
        // どの記憶ともマッチしなかったので、新規登録
        createNewObject(candidates[i], now);
      }
    }

    // ==========================================
    // STEP 4: 「幽霊チェック」 (Rules)
    // ==========================================
    // リストの後ろからループして安全に削除する
    for (int i = object_list_.size() - 1; i >= 0; --i)
    {
      // すでに安定しているものは消さない
      if (object_list_[i].stable) continue; 

      // この「不安定」な物体が、他の「安定」物体の近くにないかチェック
      bool is_ghost = false;
      for (const auto &other : object_list_) {
        if (object_list_[i].id == other.id) continue; // IDが同じなら自分自身なのでスキップ
        if (!other.stable) continue; // 相手が安定している場合のみチェック

        float d = getDist(object_list_[i].cx, object_list_[i].cy, other.cx, other.cy);
        if (d < ghost_dist_) {
          is_ghost = true; // 近すぎる！これはノイズだ
          break;
        }
      }

      // ノイズなら削除
      if (is_ghost) {
        object_list_.erase(object_list_.begin() + i);
      }
    }

    // ==========================================
    // STEP 5: 時間経過の処理 (Age & Delete)
    // ==========================================
    auto it = object_list_.begin();
    while (it != object_list_.end())
    {
      double age = (now - it->first_seen).seconds();
      double unseen_time = (now - it->last_seen).seconds();

      // 1. 安定化判定 (例: 2秒以上存在したら安定)
      if (!it->stable && age > stable_time_) {
        it->stable = true;
      }

      // 2. 忘却判定
      // 安定しているなら長く覚える、不安定ならすぐ忘れる
      double timeout = it->stable ? memory_time_ : 0.5;

      if (unseen_time > timeout) {
        it = object_list_.erase(it); // さようなら
      } else {
        ++it;
      }
    }

    // ==========================================
    // STEP 6: 可視化 (Visualize)
    // ==========================================
    publishMarkers(now);
  }

  // ---------------------------------------------------------
  // ロジック関数群
  // ---------------------------------------------------------

  void createNewObject(const RawCandidate &c, rclcpp::Time now) {
    TrackedObject obj;
    obj.id = next_obj_id_++;
    obj.cx = c.cx; obj.cy = c.cy;
    obj.min_x = c.min_x; obj.max_x = c.max_x;
    obj.min_y = c.min_y; obj.max_y = c.max_y;
    obj.width = c.width; obj.height = c.height;
    obj.first_seen = now;
    obj.last_seen = now;
    obj.stable = false;
    
    object_list_.push_back(obj);
  }

  void updateObject(TrackedObject &obj, const RawCandidate &c, rclcpp::Time now) {
    // 位置は新しい情報に更新
    obj.cx = c.cx;
    obj.cy = c.cy;
    
    // 【重要】サイズは「過去最大」を維持する (maxをとる)
    // これにより、箱の一部しか見えなくなってもサイズが小さくならない
    obj.width = std::max(obj.width, c.width);
    obj.height = std::max(obj.height, c.height);
    
    obj.min_x = std::min(obj.min_x, c.min_x);
    obj.max_x = std::max(obj.max_x, c.max_x);
    obj.min_y = std::min(obj.min_y, c.min_y);
    obj.max_y = std::max(obj.max_y, c.max_y);

    obj.last_seen = now; // 「今見たよ」と更新
  }

  // 生の点群データからクラスター（候補）を作る処理（以前のロジックと同じ）
  std::vector<RawCandidate> extractCandidates(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<RawCandidate> candidates;
    
    // ポイント抽出
    std::vector<std::pair<float, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      points.emplace_back(*iter_x, *iter_y);
    if (points.empty()) return candidates;

    // クラスタリング
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current;
    current.push_back(points.front());
    
    for (size_t i = 1; i < points.size(); ++i) {
      float dx = points[i].first - points[i-1].first;
      float dy = points[i].second - points[i-1].second;
      if (std::sqrt(dx*dx + dy*dy) > cluster_dist_) {
        if (current.size() >= (size_t)min_pts_) clusters.push_back(current);
        current.clear();
      }
      current.push_back(points[i]);
    }
    if (current.size() >= (size_t)min_pts_) clusters.push_back(current);

    // 候補リスト作成
    for (const auto &cluster : clusters) {
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (const auto &p : cluster) {
            min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
            min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
        }
        float w = max_x - min_x;
        float h = max_y - min_y;

        // ゴミ除去フィルタ（小さすぎるものはここで捨てる）
        if (w < 0.05 && h < 0.05) continue; 

        candidates.push_back({(min_x+max_x)/2.0f, (min_y+max_y)/2.0f, min_x, max_x, min_y, max_y, w, h});
    }
    return candidates;
  }

  void publishMarkers(rclcpp::Time now) {
    visualization_msgs::msg::MarkerArray ma;
    for(const auto &obj : object_list_) {
      // チラつき防止：不安定かつ生まれてすぐのものは表示しない
      if (!obj.stable && (now - obj.first_seen).seconds() < 0.3) continue;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; // ※必要に応じて "base_link" 等に変更してください
      m.header.stamp = now;
      m.ns = "objects";
      m.id = obj.id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = obj.cx;
      m.pose.position.y = obj.cy;
      m.pose.position.z = 0.2;
      m.scale.x = std::max(0.1f, obj.width);
      m.scale.y = std::max(0.1f, obj.height);
      m.scale.z = 0.4;
      m.lifetime = rclcpp::Duration::from_seconds(0.2); // 自動消去用
      
      if(obj.stable) {
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8; // 緑
      } else {
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5; // 黄
      }
      
      // IDを表示するテキストマーカー
      visualization_msgs::msg::Marker text = m;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.id = obj.id + 1000;
      text.text = "ID " + std::to_string(obj.id);
      text.pose.position.z += 0.5;
      text.scale.z = 0.2;
      text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;

      ma.markers.push_back(m);
      ma.markers.push_back(text);
    }
    marker_pub_->publish(ma);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTrackerNode>());
  rclcpp::shutdown();
  return 0;
}