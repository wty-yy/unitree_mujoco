#include "camera_height_map.h"
#include <iostream>

void require(bool condition, const char* message) { if(!condition) throw std::runtime_error(message); }

int main() {
    char error[1024];
    const char* xml=R"(<mujoco><worldbody><body name="robot" pos="0 0 .35"><freejoint/><geom size=".05" mass="1" contype="0" conaffinity="0"/></body><geom name="floor" type="plane" size="10 10 .1"/><geom name="box" type="box" pos="1.5 0 .3" size=".5 .6 .3"/></worldbody></mujoco>)";
    std::unique_ptr<mjSpec,decltype(&mj_deleteSpec)> spec(mj_parseXMLString(xml,nullptr,error,sizeof(error)),mj_deleteSpec);
    require(bool(spec),error);
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> m(mj_compile(spec.get(),nullptr),mj_deleteModel);
    require(bool(m),error);
    std::unique_ptr<mjData,decltype(&mj_deleteData)> d(mj_makeData(m.get()),mj_deleteData);
    mj_forward(m.get(),d.get());
    CameraHeightMapConfig c;c.training_augmentation=false;
    CameraHeightMap map(m.get(),17,-.5,c);
    require(map.update(d.get()),"first acquisition missing");
    require(!map.update(d.get()),"same camera frame acquired twice");
    require(map.hits().size()>10,"camera acquired no terrain");
    require(!map.query(1.5,0).second,"hidden box top leaked into memory");
    CameraHeightMap::Vec observed=map.hits().front();
    require(map.query(observed.x(),observed.y()).second,"first hit not stored");
    std::vector<CameraHeightMap::DebugPoint> points;std::vector<CameraHeightMap::DebugLine> lines;
    map.debug(d.get(),points,lines);require(lines.size()==76,"range or input boundary lines missing");
    int layers[4]={};for(const auto& p:points)++layers[p.layer];
    require(layers[0]&&layers[1]&&layers[2]&&layers[3],"expected debug state layer missing");
    const auto memory=map.memory();map.debug(d.get(),points,lines);require(memory==map.memory(),"debug geometry changed observation memory");
    d->time=.08;d->qpos[3]=0;d->qpos[6]=1;mj_forward(m.get(),d.get());map.update(d.get());
    require(map.query(observed.x(),observed.y()).second,"turning discarded observed world height");
    d->time=.16;d->qpos[0]=10;mj_forward(m.get(),d.get());map.update(d.get());
    require(!map.query(observed.x(),observed.y()).second,"moving window retained out-of-window terrain");
    d->time=0;d->qpos[0]=0;d->qpos[3]=1;d->qpos[6]=0;mj_forward(m.get(),d.get());map.update(d.get());
    require(map.frame_count()==1,"rewinding simulation did not reset acquisition");
    for(size_t i=0;i<map.output().missing.size();++i) {
        require(map.output().missing[i]==0||map.output().missing[i]==1,"nonbinary missing mask");
        require(map.output().missing[i]==0||map.output().elevation[i]==0,"unknown height is not zero");
    }
    CameraHeightMapConfig delay;c=delay;c.training_augmentation=true;c.noise_std=0;c.bias_range=0;c.drift_range=0;
    c.correlated_noise_std=0;c.tilt_range=0;c.dropout_probability=0;c.outlier_probability=0;c.full_visibility_probability=0;c.max_delay_steps=2;
    // Use the clean ground behind the box for a scalar reference across frames.
    d->qpos[0]=-3;d->qpos[2]=.35;d->time=0;mj_forward(m.get(),d.get());
    CameraHeightMap delayed(m.get(),17,-.5,c);delayed.update(d.get());
    const int pixel=11*11+5;require(delayed.output().missing[pixel]==0,"delay reference cell unknown");
    require(delayed.delay()>=1&&delayed.delay()<=2,"delay not in training range");
    for(int frame=1;frame<=5;++frame) {
        d->time=frame/15.+1e-7;d->qpos[2]=.35+.02*frame;mj_forward(m.get(),d.get());delayed.update(d.get());
        const double expected=-.35-.02*std::max(0,frame-delayed.delay());
        require(std::abs(delayed.output().elevation[pixel]-expected)<1e-5,"delayed height/mask history misaligned");
    }
    c.noise_std=20;c.max_delay_steps=0;d->time=0;mj_forward(m.get(),d.get());
    CameraHeightMap unclipped(m.get(),17,-.5,c);unclipped.update(d.get());
    require(std::any_of(unclipped.output().elevation.begin(),unclipped.output().elevation.end(),[](float v){return std::abs(v)>2;}),"height_clip null was clipped");
    c.height_clip={-2,2};CameraHeightMap clipped(m.get(),17,-.5,c);clipped.update(d.get());
    for(float v:clipped.output().elevation)require(std::isfinite(v)&&std::abs(v)<=2,"configured height clipping failed");
    c=CameraHeightMapConfig{};c.full_visibility_probability=1;c.max_delay_steps=0;c.dropout_probability=1;c.noise_std=0;
    CameraHeightMap full(m.get(),17,-.5,c);full.update(d.get());require(full.full_view(),"full-view episode not selected");
    for(float v:full.output().missing)require(v==0,"full-view dropout not suppressed");
    std::cout<<"Camera geometry, occlusion, memory, debug layers, reset, delay, clipping and full-view checks passed\n";
}
