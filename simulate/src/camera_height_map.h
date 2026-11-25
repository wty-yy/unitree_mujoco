#pragma once

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

// In lab.xml group 4 is the roof. Keep it out of height-map rays.
inline bool isHeightMapTerrain(const mjModel* model, int geom) {
    return model->body_weldid[model->geom_bodyid[geom]] == 0 &&
           model->geom_group[geom] != 4 &&
           (model->geom_contype[geom] || model->geom_conaffinity[geom]);
}

// Mirrors go2_rl_lab/elev_utils/{projection,memory,augmentation,observations,display}.
struct CameraHeightMapConfig {
    bool enabled = false;
    bool debug = true;
    bool training_augmentation = true;
    unsigned seed = 42;
    std::array<double, 3> camera_position{0.315264242, 0.000000004, 0.101813775};
    std::array<double, 3> camera_mount_rpy{0, 0, 0};
    double camera_pitch_deg = 32, horizontal_fov_deg = 89, vertical_fov_deg = 58;
    double near = 0.16, far = 2, camera_fps = 15, reference_height = 0.35;
    int cache_size = 50;
    double resolution = 0.1;
    double noise_std = 0.01, bias_range = 0.01, drift_range = 0.03;
    double correlated_noise_std = 0.01, tilt_range = 0.03;
    double dropout_probability = 0.02, outlier_probability = 0.01;
    double full_visibility_probability = 0.1;
    int max_delay_steps = 4;
    int debug_interval = 5; // Training control steps (0.02 s), independent of acquisition.
    // The current four-skill export uses height_clip: null.
    std::array<double, 2> height_clip{-INFINITY, INFINITY};

    void load(const YAML::Node& n) {
        if (!n || n.IsNull()) return;
#define READ_CAMERA_FIELD(name) name = n[#name].as<decltype(name)>(name)
        READ_CAMERA_FIELD(enabled); READ_CAMERA_FIELD(debug); READ_CAMERA_FIELD(training_augmentation);
        READ_CAMERA_FIELD(seed); READ_CAMERA_FIELD(camera_pitch_deg); READ_CAMERA_FIELD(horizontal_fov_deg);
        READ_CAMERA_FIELD(vertical_fov_deg); READ_CAMERA_FIELD(camera_fps); READ_CAMERA_FIELD(reference_height);
        READ_CAMERA_FIELD(cache_size); READ_CAMERA_FIELD(resolution); READ_CAMERA_FIELD(noise_std);
        READ_CAMERA_FIELD(bias_range); READ_CAMERA_FIELD(drift_range); READ_CAMERA_FIELD(correlated_noise_std);
        READ_CAMERA_FIELD(tilt_range); READ_CAMERA_FIELD(dropout_probability); READ_CAMERA_FIELD(outlier_probability);
        READ_CAMERA_FIELD(full_visibility_probability); READ_CAMERA_FIELD(max_delay_steps);
        READ_CAMERA_FIELD(debug_interval);
#undef READ_CAMERA_FIELD
        const auto vector = [&](const char* key, auto& dest) {
            if (!n[key] || n[key].IsNull()) return;
            const auto value = n[key].as<std::vector<double>>();
            if (value.size() != dest.size()) throw std::invalid_argument(std::string(key) + ": wrong number of values");
            std::copy(value.begin(), value.end(), dest.begin());
        };
        vector("camera_position", camera_position); vector("camera_mount_rpy", camera_mount_rpy);
        std::array<double, 2> range{near, far}; vector("depth_range", range); near = range[0]; far = range[1];
        vector("height_clip", height_clip);
    }
};

class CameraHeightMap {
public:
    using Vec = Eigen::Vector3d;
    using Mat = Eigen::Matrix3d;
    struct Frame { std::vector<float> elevation, missing; };
    struct DebugPoint { Vec position; int layer; }; // unknown, measured, candidates, input
    struct DebugLine { Vec from, to; int layer; }; // input, camera

    CameraHeightMap(const mjModel* model, int width, double origin_x, CameraHeightMapConfig config)
        : model_(mj_copyModel(nullptr, model), mj_deleteModel), cfg_(config), width_(width), origin_x_(origin_x),
          rng_(config.seed) {
        validate();
        heights_.assign(cfg_.cache_size * cfg_.cache_size, -INFINITY);
        errors_.assign(heights_.size(), -INFINITY); measured_.assign(heights_.size(), false);
        for (int i = 0; i < model->ngeom; ++i) {
            model_->geom_group[i] = isHeightMapTerrain(model, i) ? 0 : 1;
        }
        const auto& r = cfg_.camera_mount_rpy;
        camera_rotation_ = (Eigen::AngleAxisd(r[2], Vec::UnitZ()) * Eigen::AngleAxisd(r[1], Vec::UnitY()) *
                            Eigen::AngleAxisd(r[0], Vec::UnitX()) *
                            Eigen::AngleAxisd(cfg_.camera_pitch_deg * pi / 180, Vec::UnitY())).toRotationMatrix();
        tan_h_ = std::tan(cfg_.horizontal_fov_deg * pi / 360);
        tan_v_ = std::tan(cfg_.vertical_fov_deg * pi / 360);
        reset();
    }

    void reset() {
        std::fill(heights_.begin(), heights_.end(), -INFINITY);
        std::fill(errors_.begin(), errors_.end(), -INFINITY);
        initialized_ = false; last_frame_ = -1; last_time_ = -INFINITY; event_time_ = NAN;
        frames_.clear(); hits_.clear();
        output_.elevation.assign(width_ * 11, 0); output_.missing.assign(width_ * 11, 1);
        full_view_ = cfg_.training_augmentation && uniform() < cfg_.full_visibility_probability;
        bias_ = cfg_.training_augmentation ? symmetric(cfg_.bias_range) : 0;
        for (int i = 0; i < 2; ++i) {
            drift_[i] = cfg_.training_augmentation ? symmetric(cfg_.drift_range) : 0;
            tilt_[i] = cfg_.training_augmentation ? symmetric(cfg_.tilt_range) : 0;
        }
        delay_ = cfg_.training_augmentation && cfg_.max_delay_steps > 0 ? integer(1, cfg_.max_delay_steps) : 0;
        frame_count_ = 0;
    }

    bool update(const mjData* data) {
        if (data->time < last_time_) reset();
        last_time_ = data->time;
        const long frame = static_cast<long>(std::floor(data->time * cfg_.camera_fps + 1e-8));
        if (frame == last_frame_) return false;
        last_frame_ = frame;
        pose(data);
        acquire(data);
        const double elapsed = std::isfinite(event_time_) ? std::max(0.0, data->time - event_time_) : 0;
        event_time_ = data->time;
        const bool events = cfg_.training_augmentation && !full_view_;
        const bool corrupt = events && uniform() < -std::expm1(-elapsed / 10);
        const bool forget_patch = events && uniform() < -std::expm1(-elapsed / 5);
        const bool forget_all = events && uniform() < -std::expm1(-elapsed / 30);
        if (forget_all) {
            std::fill(heights_.begin(), heights_.end(), -INFINITY);
            std::fill(errors_.begin(), errors_.end(), -INFINITY);
        }
        integrate();
        for (size_t i = 0; i < heights_.size(); ++i)
            if (measured_[i] || !std::isfinite(heights_[i])) errors_[i] = -INFINITY;
        if (forget_patch && !forget_all) for (int i : patch()) heights_[i] = errors_[i] = -INFINITY;
        const bool occupied = std::any_of(errors_.begin(), errors_.end(), [](float v) { return std::isfinite(v); });
        if (corrupt && !occupied && !forget_patch && !forget_all) {
            const auto selected = patch();
            const double offset = 0.1 + 1.9 * uniform();
            const bool extreme = uniform() < 0.1;
            for (int i : selected) errors_[i] = extreme ? position_.z() + 2 : heights_[i] + offset;
        }
        auto latest = observation(data);
        if (frames_.empty()) frames_.assign(cfg_.max_delay_steps + 1, latest);
        else { frames_.push_front(std::move(latest)); frames_.pop_back(); }
        output_ = frames_[delay_];
        ++frame_count_;
        return true;
    }

    const Frame& output() const { return output_; }
    const std::vector<Vec>& hits() const { return hits_; }
    const std::vector<float>& memory() const { return heights_; }
    const std::vector<float>& corruption() const { return errors_; }
    int delay() const { return delay_; }
    bool full_view() const { return full_view_; }
    long frame_count() const { return frame_count_; }
    const CameraHeightMapConfig& config() const { return cfg_; }

    // Also used by numeric conformance tests. World coordinates never expose unseen heights.
    std::pair<float, bool> query(double wx, double wy, bool augmented = false) const {
        const int x = cell(wx), y = cell(wy);
        if (!inside(x, y) || !initialized_) return {0, false};
        const int i = slot(x, y);
        const float h = augmented && std::isfinite(errors_[i]) ? errors_[i] : heights_[i];
        return std::isfinite(h) ? std::make_pair(h, true) : std::make_pair(0.f, false);
    }

    std::pair<float, bool> interpolate(double wx, double wy, bool augmented = true) const {
        const double x = wx / cfg_.resolution, y = wy / cfg_.resolution;
        const int ix = std::floor(x), iy = std::floor(y);
        const double fx = x - ix, fy = y - iy;
        double weighted = 0, total = 0;
        for (int dx = 0; dx < 2; ++dx) for (int dy = 0; dy < 2; ++dy) {
            const auto v = query((ix + dx) * cfg_.resolution, (iy + dy) * cfg_.resolution, augmented);
            const double weight = (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy);
            if (v.second) { weighted += v.first * weight; total += weight; }
        }
        return total > 1e-6 ? std::make_pair(float(weighted / total), true) : std::make_pair(0.f, false);
    }

    void debug(const mjData* data, std::vector<DebugPoint>& points, std::vector<DebugLine>& lines) {
        pose(data); points.clear(); lines.clear();
        const int half = cfg_.cache_size / 2;
        const int cx = cell(position_.x()), cy = cell(position_.y());
        for (int x = cx - half; x < cx - half + cfg_.cache_size; ++x)
            for (int y = cy - half; y < cy - half + cfg_.cache_size; ++y) {
                const double wx = x * cfg_.resolution, wy = y * cfg_.resolution;
                const auto measured = query(wx, wy);
                const auto projected = ray(data, Vec(wx + cfg_.resolution * .001, wy + cfg_.resolution * .001,
                                                      position_.z() + 20), Vec(0, 0, -1), 100);
                if (measured.second) points.push_back({Vec(wx, wy, measured.first), 1});
                else if (projected.allFinite()) points.push_back({Vec(wx, wy, projected.z()), in_camera(projected) ? 2 : 0});
            }
        for (int x = 0; x < width_; ++x) for (int y = 0; y < 11; ++y) {
            const int i = x * 11 + y;
            if (output_.missing[i] == 0) {
                Vec p = world(origin_x_ + .1 * x, -.5 + .1 * y);
                p.z() = position_.z() + output_.elevation[i]; points.push_back({p, 3});
            }
        }
        const std::array<Vec, 4> bounds{world(origin_x_, -.5), world(origin_x_ + .1 * (width_ - 1), -.5),
                                        world(origin_x_ + .1 * (width_ - 1), .5), world(origin_x_, .5)};
        for (int i = 0; i < 4; ++i) {
            Vec a = bounds[i], b = bounds[(i + 1) % 4]; a.z() = b.z() = position_.z() - cfg_.reference_height;
            lines.push_back({a, b, 0});
        }
        const std::array<Vec, 4> corners{Vec(1,-tan_h_,-tan_v_), Vec(1,tan_h_,-tan_v_),
                                         Vec(1,tan_h_,tan_v_), Vec(1,-tan_h_,tan_v_)};
        const auto camera_world = [&](const Vec& v) -> Vec { return camera_origin_ + rotation_ * camera_rotation_ * v; };
        for (int i = 0; i < 4; ++i) {
            const Vec a = corners[i], b = corners[(i + 1) % 4];
            for (int j = 0; j < 16; ++j) {
                const Vec u = a + (b-a) * (double(j)/16), v = a + (b-a) * (double(j+1)/16);
                lines.push_back({camera_world(u.normalized()*cfg_.far), camera_world(v.normalized()*cfg_.far), 1});
            }
            lines.push_back({camera_world(a*cfg_.near), camera_world(b*cfg_.near), 1});
            lines.push_back({camera_world(a*cfg_.near), camera_world(a.normalized()*cfg_.far), 1});
        }
    }

private:
    static constexpr double pi = 3.14159265358979323846;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_;
    CameraHeightMapConfig cfg_;
    int width_; double origin_x_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0,1};
    std::vector<float> heights_, errors_;
    std::vector<bool> measured_;
    std::vector<Vec> hits_;
    std::deque<Frame> frames_;
    Frame output_;
    Vec position_, camera_origin_;
    Mat rotation_, camera_rotation_;
    double yaw_ = 0, tan_h_ = 0, tan_v_ = 0, bias_ = 0;
    std::array<double,2> drift_{}, tilt_{};
    int center_x_ = 0, center_y_ = 0, delay_ = 0;
    bool initialized_ = false, full_view_ = false;
    long last_frame_ = -1, frame_count_ = 0;
    double last_time_ = -INFINITY, event_time_ = NAN;

    void validate() const {
        const auto finite = [](double x) { return std::isfinite(x); };
        if (cfg_.cache_size < 3 || cfg_.cache_size > 1000 || !finite(cfg_.resolution) || cfg_.resolution <= 0 ||
            !finite(cfg_.near) || !finite(cfg_.far) || cfg_.near < 0 || cfg_.far <= cfg_.near ||
            !finite(cfg_.camera_fps) || cfg_.camera_fps <= 0 || !finite(cfg_.camera_pitch_deg) ||
            !(cfg_.horizontal_fov_deg > 0 && cfg_.horizontal_fov_deg < 180) ||
            !(cfg_.vertical_fov_deg > 0 && cfg_.vertical_fov_deg < 180) ||
            !finite(cfg_.reference_height) || cfg_.reference_height <= 0 || cfg_.max_delay_steps < 0 ||
            cfg_.max_delay_steps > 1000 || cfg_.debug_interval < 1 || !(cfg_.height_clip[0] < cfg_.height_clip[1]) ||
            (width_ != 17 && width_ != 9) || !finite(origin_x_))
            throw std::invalid_argument("Invalid camera height-map geometry or delay");
        for (double p : {cfg_.dropout_probability, cfg_.outlier_probability, cfg_.full_visibility_probability})
            if (!finite(p) || p < 0 || p > 1) throw std::invalid_argument("Invalid height-map probability");
        for (double p : {cfg_.noise_std, cfg_.bias_range, cfg_.drift_range, cfg_.correlated_noise_std, cfg_.tilt_range})
            if (!finite(p) || p < 0) throw std::invalid_argument("Invalid height-map noise range");
        for (double p : cfg_.camera_position) if (!finite(p)) throw std::invalid_argument("Invalid camera mount");
        for (double p : cfg_.camera_mount_rpy) if (!finite(p)) throw std::invalid_argument("Invalid camera mount rotation");
        const double radius = std::hypot(std::max(std::abs(origin_x_), std::abs(origin_x_ + .1*(width_-1))), .5)
                            + cfg_.drift_range * std::sqrt(2.);
        const int margin = std::min(cfg_.cache_size/2, cfg_.cache_size-cfg_.cache_size/2-1);
        if (radius + cfg_.resolution/2 > margin * cfg_.resolution)
            throw std::invalid_argument("Camera cache does not cover the policy grid and drift");
    }

    double uniform() { return std::generate_canonical<double, 53>(rng_); }
    double symmetric(double range) { return (2 * uniform() - 1) * range; }
    int integer(int low, int high) { return std::uniform_int_distribution<int>(low, high)(rng_); }
    int cell(double x) const { return static_cast<int>(std::floor(x / cfg_.resolution + .5)); }
    int modulo(int x) const { return (x % cfg_.cache_size + cfg_.cache_size) % cfg_.cache_size; }
    int slot(int x, int y) const { return modulo(x) * cfg_.cache_size + modulo(y); }
    bool inside(int x, int y) const {
        const int half = cfg_.cache_size / 2;
        return x >= center_x_-half && x < center_x_-half+cfg_.cache_size &&
               y >= center_y_-half && y < center_y_-half+cfg_.cache_size;
    }
    void pose(const mjData* d) {
        position_ = Eigen::Map<const Vec>(d->qpos);
        mjtNum matrix[9]; mju_quat2Mat(matrix, d->qpos + 3);
        rotation_ = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(matrix);
        yaw_ = std::atan2(rotation_(1,0), rotation_(0,0));
        camera_origin_ = position_ + rotation_ * Eigen::Map<const Vec>(cfg_.camera_position.data());
    }
    Vec world(double x, double y) const {
        return position_ + Vec(std::cos(yaw_)*x-std::sin(yaw_)*y, std::sin(yaw_)*x+std::cos(yaw_)*y, 0);
    }
    bool in_camera(const Vec& point) const {
        const Vec delta = point - camera_origin_;
        const Vec local = camera_rotation_.transpose() * rotation_.transpose() * delta;
        return point.allFinite() && local.x() >= cfg_.near && local.x() <= cfg_.far &&
               std::abs(local.y()) <= local.x()*tan_h_ && std::abs(local.z()) <= local.x()*tan_v_ &&
               delta.norm() <= cfg_.far;
    }
    Vec ray(const mjData* d, const Vec& origin, const Vec& direction, double max_range) const {
        const mjtByte groups[6] = {1,0,0,0,0,0}; int geom = -1;
        const double distance = mj_ray(model_.get(), d, origin.data(), direction.data(), groups, 1, -1, &geom, nullptr);
        if (distance < 0 || distance > max_range) return Vec::Constant(NAN);
        return origin + distance * direction;
    }
    void acquire(const mjData* d) {
        hits_.clear();
        const int half = cfg_.cache_size/2, cx = cell(position_.x()), cy = cell(position_.y());
        for (int x = cx-half; x < cx-half+cfg_.cache_size; ++x)
            for (int y = cy-half; y < cy-half+cfg_.cache_size; ++y) {
                // Terrain projections guide rays; only the later first hits enter memory.
                const Vec target = ray(d, Vec(x*cfg_.resolution+cfg_.resolution*.001,
                                             y*cfg_.resolution+cfg_.resolution*.001, position_.z()+20),
                                       Vec(0,0,-1), 100);
                if (in_camera(target)) {
                    const Vec hit = ray(d, camera_origin_, (target-camera_origin_).normalized(), cfg_.far);
                    if (in_camera(hit)) hits_.push_back(hit);
                }
            }
        for (int u=0; u<32; ++u) for (int v=0; v<24; ++v) {
            const Vec local(1, ((u+.5)/32*2-1)*tan_h_, ((v+.5)/24*2-1)*tan_v_);
            const Vec hit = ray(d, camera_origin_, rotation_*camera_rotation_*local.normalized(), cfg_.far);
            if (in_camera(hit)) hits_.push_back(hit);
        }
    }
    void integrate() {
        const int nx=cell(position_.x()), ny=cell(position_.y()), half=cfg_.cache_size/2;
        for (int x=nx-half; x<nx-half+cfg_.cache_size; ++x)
            for (int y=ny-half; y<ny-half+cfg_.cache_size; ++y)
                if (!initialized_ || !inside(x,y)) heights_[slot(x,y)]=-INFINITY;
        center_x_=nx; center_y_=ny; initialized_=true;
        std::fill(measured_.begin(), measured_.end(), false);
        for (const auto& hit : hits_) {
            const int x=cell(hit.x()), y=cell(hit.y());
            if (inside(x,y)) {
                const int i=slot(x,y); heights_[i]=std::max(heights_[i],float(hit.z())); measured_[i]=true;
            }
        }
    }
    std::vector<int> patch() {
        std::vector<int> eligible;
        for (size_t i=0; i<heights_.size(); ++i) if (std::isfinite(heights_[i]) && !measured_[i]) eligible.push_back(i);
        if (eligible.empty()) return {};
        const int i=eligible[integer(0,eligible.size()-1)];
        const int lower_x=center_x_-cfg_.cache_size/2, lower_y=center_y_-cfg_.cache_size/2;
        const int x=lower_x+modulo(i/cfg_.cache_size-lower_x), y=lower_y+modulo(i%cfg_.cache_size-lower_y);
        const int low=std::max(1,int(std::ceil(.2/cfg_.resolution-1e-6)));
        const int high=std::max(low,int(std::floor(.6/cfg_.resolution+1e-6)));
        const int sx=integer(low,high), sy=integer(low,high), start_x=x-sx/2, start_y=y-sy/2;
        std::vector<int> result;
        for (int px=start_x; px<start_x+sx; ++px) for (int py=start_y; py<start_y+sy; ++py)
            if (inside(px,py)) { const int j=slot(px,py); if (std::isfinite(heights_[j])&&!measured_[j]) result.push_back(j); }
        return result;
    }
    static double bilinear(const std::vector<float>& v, int width, int height, double x, double y) {
        x=std::clamp(x,0.,double(width-1)); y=std::clamp(y,0.,double(height-1));
        const int ix=std::floor(x), iy=std::floor(y), jx=std::min(ix+1,width-1), jy=std::min(iy+1,height-1);
        const double fx=x-ix, fy=y-iy;
        return (v[iy*width+ix]*(1-fx)+v[iy*width+jx]*fx)*(1-fy)
             + (v[jy*width+ix]*(1-fx)+v[jy*width+jx]*fx)*fy;
    }
    Frame observation(const mjData* d) {
        Frame frame; frame.elevation.resize(width_*11); frame.missing.resize(width_*11);
        std::vector<float> truth, truth_valid;
        if (full_view_) {
            truth.resize(width_*11); truth_valid.resize(width_*11);
            for (int y=0;y<11;++y) for (int x=0;x<width_;++x) {
                const Vec p=world(origin_x_+.1*x,-.5+.1*y);
                const Vec h=ray(d,Vec(p.x(),p.y(),position_.z()+20),Vec(0,0,-1),100);
                truth[y*width_+x]=h.allFinite()?h.z()-position_.z():0; truth_valid[y*width_+x]=h.allFinite();
            }
        }
        std::vector<float> coarse(12,0);
        if(cfg_.training_augmentation) for(auto& v:coarse) v=normal_(rng_);
        for(int x=0;x<width_;++x) for(int y=0;y<11;++y) {
            const int i=x*11+y; const double lx=origin_x_+.1*x, ly=-.5+.1*y;
            const Vec p=world(lx+drift_[0],ly+drift_[1]);
            const auto sample=interpolate(p.x(),p.y());
            double value=sample.second?sample.first-position_.z():0; bool valid=sample.second;
            if(full_view_) {
                value=bilinear(truth,width_,11,x+drift_[0]/.1,y+drift_[1]/.1);
                valid=bilinear(truth_valid,width_,11,x+drift_[0]/.1,y+drift_[1]/.1)>.999;
            }
            if(cfg_.training_augmentation) {
                value+=bias_+tilt_[0]*lx+tilt_[1]*ly+normal_(rng_)*cfg_.noise_std;
                // PyTorch interpolate(..., align_corners=False), NCHW source 3x4.
                value+=bilinear(coarse,4,3,(x+.5)*4/width_-.5,(y+.5)*3/11-.5)*cfg_.correlated_noise_std;
                valid=valid&&(full_view_||uniform()>=cfg_.dropout_probability);
                if(valid&&uniform()<cfg_.outlier_probability) value+=normal_(rng_);
            }
            frame.elevation[i]=valid?std::clamp(value,cfg_.height_clip[0],cfg_.height_clip[1]):0;
            frame.missing[i]=valid?0:1;
        }
        return frame;
    }
};
